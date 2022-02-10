#!/usr/bin/python3
import numpy as np
import time

# for ros
import rospy
from obstacle_detector.msg import Obstacles
from nav_msgs.msg import Odometry

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
        self.obstacles_sub = rospy.Subscriber("obstacles", Obstacles, self.obstacles_sub_callback)
        # self.rate = rospy.Rate(30)
        
    def ekf_localization(self, v, w):
        # set motion covariance
        a1 = 0.01
        a2 = 0.01
        a3 = 0.01
        a4 = 0.01
        # for convenience, ex: s_dt = sin(theta+wdt) 
        theta = self.mu_past[2, 0].copy()
        s = np.sin(theta)
        c = np.cos(theta)
        s_dt = np.sin(theta + w*self.dt)
        c_dt = np.cos(theta + w*self.dt)
        
        # ekf predict step:
        if w == 0:
            G = np.array([[1, 0,  v*c],\
                          [0, 1, -v*s],\
                          [0, 0,    1]])

            V = np.array([[s, 0],\
                          [c, 0],\
                          [0, 0]])

            M = np.array([[a1*v**2 + a2*w**2,                 0],\
                          [                0, a3*v**2 + a4*w**2]])

            mu_bar = self.mu_past.copy() + np.array([[v*s],\
                                                     [v*c],\
                                                     [  0]])
        
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
        

        sigma_bar = G@self.sigma_past@np.transpose(G) + V@M@np.transpose(V)

        # ekf update step:
        if self.if_new_obstacles is True:
            # for every obstacle (or beacon pillar), check the distance between real beacon position and itself.
            for landmark_scan in np.nditer(self.beacon_scan, flags=['external_loop'], order='F'):
                landmark_scan = np.reshape(landmark_scan, (2,1))
                # transfer the coordinate of landmark_scan from base_laser_link to map
                landmark_scan = self._tf_laser_to_map(mu_bar, landmark_scan)
                min_dist = 9999
                min_index = 0
                for i in range(self.beacon_position.shape[1]):
                    landmark_i = np.reshape(self.beacon_position[:,i], (2,1))
                    q = self._euclidean_distance(landmark_scan, landmark_i)

            # for testing
            self.mu = mu_bar.copy()
            self.sigma = sigma_bar.copy()
            # print(self.sigma)
            self.mu_past = self.mu.copy()
            self.sigma_past = self.sigma.copy()
        # finish once ekf, change the flag
        self.if_new_obstacles = False

        
        # A = np.array([[],\
        #               [],\
        #               []])

    def _euclidean_distance(self, a, b):
        return np.sqrt((b[1, 0]-a[1, 0])**2 + (b[0, 0]-a[0, 0])**2)

    def _angle_limit_checking(self, theta):
        if theta > np.pi:
            theta -= 2 * np.pi
        elif theta <= -np.pi:
            theta += 2 * np.pi
        return theta

    def _tf_laser_to_map(self, mu_bar, landmark_scan):
        # rotate theta and translation (x, y) from the laser frame 
        s = np.sin(mu_bar[2,0])
        c = np.cos(mu_bar[2,0])
        landmark_scan = np.array([[c, -s],[s, c]])@landmark_scan + np.reshape(mu_bar[0:2, 0], (2,1))
        return landmark_scan

    def odom_sub_callback(self, odom):
        v = odom.twist.twist.linear.x
        w = odom.twist.twist.angular.z
        self.ekf_localization(v, w)

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

if __name__ == '__main__':
    # rospy.init_node('ekf_localization', anonymous=True)
    ekf = EKF()
    while not rospy.is_shutdown():
        rospy.spin()