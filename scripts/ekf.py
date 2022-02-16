#!/usr/bin/python3
import numpy as np
import time

# for ros
import rospy
from obstacle_detector.msg import Obstacles
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

class EKF:

    def __init__(self):
        # for beacon position ([x], [y])
        self.beacon_position = np.array([[0.05, 1.05, 1.95],\
                                         [ 3.1, -0.1,  3.1]])

        # for robot initial state (x; y; phi(-pi,pi))
        self.mu_0 = np.array([[0.5],\
                              [0.5],\
                              [0.5*np.pi]])
        
        # for robot state
        self.mu_past = self.mu_0.copy()
        # self.mu_bar = np.zeros((3,1))
        self.mu = np.array([[0.0],\
                            [0.0],\
                            [0.0]])

        self.sigma_past = np.zeros((3,3))
        # self.sigma_bar = np.zeros((3,3))
        self.sigma = np.zeros((3,3))

        self.dt = 1/50

        # for beacon pillar detection
        self.if_new_obstacles = False
        self.beacon_scan = np.nan

        # for ros
        rospy.init_node('ekf_localization', anonymous=True)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_sub_callback)
        self.obstacles_sub = rospy.Subscriber("raw_obstacles", Obstacles, self.obstacles_sub_callback)
        self.ekf_pose_pub = rospy.Publisher("ekf_pose", PoseWithCovarianceStamped, queue_size=10)
        # self.rate = rospy.Rate(30)
        
    def ekf_localization(self, v, w):
        # set motion covariance
        a1 = 0.5
        a2 = 0.8
        a3 = 0.5
        a4 = 0.8
        # set a minimum likelihood value
        mini_likelihood = 0.5
        # for convenience, ex: s_dt = sin(theta+wdt) 
        theta = self.mu_past[2, 0].copy()
        s = np.sin(theta)
        c = np.cos(theta)
        s_dt = np.sin(theta + w*self.dt)
        c_dt = np.cos(theta + w*self.dt)
        
        # ekf predict step:
        if w < 1e-5 and w > -1e-5 :
            G = np.array([[1, 0, -v*s*self.dt],\
                          [0, 1,  v*c*self.dt],\
                          [0, 0,            1]])

            V = np.array([[c*self.dt, 0],\
                          [s*self.dt, 0],\
                          [0        , 0]])

            M = np.array([[a1*v**2 + a2*w**2,                 0],\
                          [                0, a3*v**2 + a4*w**2]])

            mu_bar = self.mu_past.copy() + np.array([[v*c*self.dt],\
                                                     [v*s*self.dt],\
                                                     [          0]])
        
        else:
            G = np.array([[1, 0, (v*(-c+c_dt))/w],\
                          [0, 1, (v*(-s+s_dt))/w],\
                          [0, 0,               1]])

            V = np.array([[(-s+s_dt)/w,  v*(s-s_dt)/w**2 + v*self.dt*(c_dt)/w],\
                          [ (c-c_dt)/w, -v*(c-c_dt)/w**2 + v*self.dt*(s_dt)/w],\
                          [          0,                               self.dt]])

            M = np.array([[a1*v**2+a2*w**2,             0],\
                          [            0, a3*v**2+a4*w**2]])

            mu_bar = self.mu_past.copy() + np.array([[v*(-s+s_dt)/w],\
                                                     [ v*(c-c_dt)/w],\
                                                     [    w*self.dt]])
            mu_bar[2,0] = self._angle_limit_checking(mu_bar[2,0])
        

        sigma_bar = G@self.sigma_past@G.T + V@M@V.T
        # sigma_bar = self._fix_FP_issue(sigma_bar)
        sigma_bar[sigma_bar<1e-10] = 0
        
        # ekf update step:
        Q = np.diag((0.001, 0.2, 0.02))
        if self.if_new_obstacles is True:
            # for every obstacle (or beacon pillar), check the distance between real beacon position and itself.
            for landmark_scan in np.nditer(self.beacon_scan, flags=['external_loop'], order='F'):
                landmark_scan = np.reshape(landmark_scan, (2,1))
                # transfer landmark_scan type from (x, y) to (r, phi)
                z_i = self._cartesian_to_polar(landmark_scan, np.zeros((3,1)))
                j_max = 0
                H_j_max = 0
                S_j_max = 0
                z_j_max = 0
                # print("j_k: ")
                for k in range(self.beacon_position.shape[1]):
                    landmark_k = np.reshape(self.beacon_position[:,k], (2,1))
                    z_k, H_k = self._cartesian_to_polar(landmark_k, mu_bar, cal_H=True)
                    S_k = H_k@sigma_bar@H_k.T + Q
                    # S_k = self._fix_FP_issue(S_k)
                    S_k[S_k<1e-10] = 0
                    # print(S_k)
                    try:
                        # original 
                        # j_k = 1/np.sqrt(np.linalg.det(2*np.pi*S_k)) * np.exp(-0.5*(z_i-z_k).T@np.linalg.inv(S_k)@(z_i-z_k))
                        # ln(j_k()) version
                        j_k = -0.5*(z_i-z_k).T@np.linalg.inv(S_k)@(z_i-z_k) - np.log(np.sqrt(np.linalg.det(2*np.pi*S_k)))
                        # j_k = self._fix_FP_issue(j_k)
                        j_k[j_k<1e-10] = 0
                        if np.around(j_k, 10)[0,0] > np.around(j_max, 10):
                            j_max = j_k.copy()
                            H_j_max = H_k.copy()
                            S_j_max = S_k.copy()
                            z_j_max = z_k.copy()
                    except Exception as e:
                        rospy.logerr("%s", e)
                        # continue
                    # rospy.loginfo("H_k=%s, sigma_bar=%s, S_k=%s", H_k, sigma_bar, S_k)
                    # print("S_k-Q = ", H_k@sigma_bar@H_k.T, "H_k = ", H_k, "sigma_bar = ", sigma_bar)
                if j_max > mini_likelihood and j_max != np.nan:
                    K_i = sigma_bar@H_j_max.T@np.linalg.inv(S_j_max)
                    # print(j_max)
                    mu_bar = mu_bar + K_i@(z_i-z_j_max)
                    sigma_bar = (np.eye(3) - self._fix_FP_issue(K_i@H_j_max))@sigma_bar

        self.mu = mu_bar.copy()
        self.sigma = self._fix_FP_issue(sigma_bar.copy())
        self.mu_past = self.mu.copy()
        self.sigma_past = self.sigma.copy()
        # finish once ekf, change the flag
        self.if_new_obstacles = False

    def _euclidean_distance(self, a, b):
        return np.sqrt((b[1, 0]-a[1, 0])**2 + (b[0, 0]-a[0, 0])**2)

    def _angle_limit_checking(self, theta):
        if theta > np.pi:
            theta -= 2 * np.pi
        elif theta <= -np.pi:
            theta += 2 * np.pi
        return theta

    # find polar coordinate for point a from ref point b
    def _cartesian_to_polar(self, a, b, cal_H=False):
        q_sqrt = self._euclidean_distance(a, b)
        q = q_sqrt**2
        a_b_x = a[0, 0]-b[0, 0]
        a_b_y = a[1, 0]-b[1, 0]
        z_hat = np.array([[q_sqrt],\
                          [np.arctan2(a_b_y, a_b_x) - b[2, 0]],\
                          [1]])
        z_hat[1,0] = self._angle_limit_checking(z_hat[1,0])
        if cal_H:
            H = np.array([[-(a_b_x/q_sqrt), -(a_b_y/q_sqrt),  0],\
                          [        a_b_y/q,      -(a_b_x/q), -1],\
                          [              0,               0,  0]])
            return (z_hat, H)
        else:
            return z_hat

    def _tf_laser_to_map(self, mu_bar, landmark_scan):
        # rotate theta and translation (x, y) from the laser frame 
        s = np.sin(mu_bar[2,0])
        c = np.cos(mu_bar[2,0])
        landmark_scan = np.array([[c, -s],[s, c]])@landmark_scan + np.reshape(mu_bar[0:2, 0], (2,1))
        return landmark_scan

    def _fix_FP_issue(self, matrix, upper_bound=1e-10, lower_bound=-1e-10, positive=True):
        if positive is True:
            with np.nditer(matrix, order='C', op_flags=['readwrite']) as it:
                for x in it:
                    if x < upper_bound:
                        x[...] = 0.0
            return matrix
        else:
            with np.nditer(matrix, order='C', op_flags=['readwrite']) as it:
                for x in it:
                    if x < upper_bound and x > lower_bound:
                        x[...] = 0.0
            return matrix

    def odom_sub_callback(self, odom):
        v = odom.twist.twist.linear.x
        w = odom.twist.twist.angular.z
        self.ekf_localization(v, w)
        self.publish_ekf_pose(odom.header)

    def obstacles_sub_callback(self, obstacles):
        self.if_new_obstacles = False
        self.beacon_scan = np.nan
        for item in obstacles.circles:
            center_xy = np.array([[item.center.x],[item.center.y]])
            if self.beacon_scan is np.nan:
                self.beacon_scan = center_xy
            else:
                self.beacon_scan = np.hstack([self.beacon_scan, center_xy])
        self.if_new_obstacles = True

    def publish_ekf_pose(self, header):
        pose = PoseWithCovarianceStamped()
        pose.header = header
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = self.mu[0,0]
        pose.pose.pose.position.y = self.mu[1,0]
        quat = quaternion_from_euler(0, 0, self.mu[2,0])
        pose.pose.pose.orientation.x = quat[0]
        pose.pose.pose.orientation.y = quat[1]
        pose.pose.pose.orientation.z = quat[2]
        pose.pose.pose.orientation.w = quat[3]
        pose.pose.covariance[0] = self.sigma[0,0] # x-x
        pose.pose.covariance[1] = self.sigma[0,1] # x-y
        pose.pose.covariance[5] = self.sigma[0,2] # x-theta
        pose.pose.covariance[6] = self.sigma[1,0] # y-x
        pose.pose.covariance[7] = self.sigma[1,1] # y-y
        pose.pose.covariance[11] = self.sigma[1,2] # y-theta
        pose.pose.covariance[30] = self.sigma[2,0] # theta-x
        pose.pose.covariance[31] = self.sigma[2,1] # theta-y
        pose.pose.covariance[35] = self.sigma[2,2] # theta-theta
        self.ekf_pose_pub.publish(pose)

if __name__ == '__main__':
    # rospy.init_node('ekf_localization', anonymous=True)
    ekf = EKF()
    while not rospy.is_shutdown():
        rospy.spin()