#include "eurobot_localization/ekf.h"
using namespace std;

void Ekf::initialize() {
    // get global parameter
    nh_.param<string>("robot_name", p_robot_name_, "");
    nh_.param<double>("initial_x", p_initial_x_, 0.5);
    nh_.param<double>("initial_y", p_initial_y_, 0.5);
    nh_.param<double>("initial_theta", p_initial_theta_deg_, 90.0);

    // get local parameter
    nh_local_.param<double>("beacon_ax", p_beacon_ax_, 0.05);
    nh_local_.param<double>("beacon_ay", p_beacon_ay_, 3.1);
    nh_local_.param<double>("beacon_bx", p_beacon_bx_, 1.05);
    nh_local_.param<double>("beacon_by", p_beacon_by_, -0.1);
    nh_local_.param<double>("beacon_cx", p_beacon_cx_, 1.95);
    nh_local_.param<double>("beacon_cy", p_beacon_cy_, 3.1);
    // for beacon position in map list<double>{ax, ay, bx, by, cx, cy}
    Eigen::Vector2d beacon_a{p_beacon_ax_, p_beacon_ay_};
    Eigen::Vector2d beacon_b{p_beacon_bx_, p_beacon_by_};
    Eigen::Vector2d beacon_c{p_beacon_cx_, p_beacon_cy_};
    beacon_in_map_ = {beacon_a, beacon_b, beacon_c};
    // for debug
    update_beacon_ = {Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 0.0)};

    // for robot state
    robotstate_.mu << p_initial_x_, p_initial_y_, degToRad(p_initial_theta_deg_);
    robotstate_.sigma << 0, 0, 0, 0, 0, 0, 0, 0, 0;

    // ekf parameter
    nh_local_.param<double>("predict_cov_a1", p_a1_, 1.5);
    nh_local_.param<double>("predict_cov_a2", p_a2_, 2.5);
    nh_local_.param<double>("predict_cov_a3", p_a3_, 1.5);
    nh_local_.param<double>("predict_cov_a4", p_a4_, 2.5);

    nh_local_.param<double>("update_cov_1", p_Q1_, 0.01);
    nh_local_.param<double>("update_cov_2", p_Q2_, 0.01);
    nh_local_.param<double>("update_cov_3", p_Q3_, 0.02);
    Q_ = Eigen::Vector3d{p_Q1_, p_Q2_, p_Q3_}.asDiagonal();
    // use log(j_k)
    nh_local_.param<double>("mini_likelihood", p_mini_likelihood_, -10000.0);
    nh_local_.param<double>("mini_likelihood_update", p_mini_likelihood_update_, 0.4);
    // use j_k
    // mini_likelihood_ = 0.0;
    // mini_likelihood_update_ = 25.0;

    // for obstacle filtering
    nh_local_.param<double>("max_obstacle_distance", p_max_obstacle_distance_, 0.5);

    // for beacon piller detection
    if_new_obstacles_ = false;
    beacon_from_scan_ = {};

    // for ros
    odom_sub_ = nh_.subscribe("odom", 50, &Ekf::odomCallback, this);
    raw_obstacles_sub_ = nh_.subscribe("raw_obstacles", 10, &Ekf::obstaclesCallback, this);
    pose_estimate_sub_ = nh_.subscribe("initialpose", 10, &Ekf::poseEstimateCallback, this);
    ekf_reset_srv_ = nh_.advertiseService("reset_ekf", &Ekf::resetEkf, this);
    ekf_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("ekf_pose", 10);
    update_beacon_pub_ = nh_.advertise<obstacle_detector::Obstacles>("update_beacon", 10);

    // for time calculate
    count_ = 0;
    duration_ = 0.0;
}

void Ekf::predict_diff(double v, double w) {
    double theta = robotstate_.mu(2);
    double s = sin(theta);
    double c = cos(theta);
    double s_dt = sin(theta + w * dt_);
    double c_dt = cos(theta + w * dt_);
    // cout << "sin= " << s << ",cos= " << c << ", theta= " << theta << endl;

    // ekf predict step
    Eigen::Matrix3d G;
    Eigen::Matrix<double, 3, 2> V;
    Eigen::Matrix2d M;

    if ((w < 0.00001) && (w > -0.00001)) {
        G << 1.0, 0.0, -v * s * dt_,
            0.0, 1.0, c * v * dt_,
            0.0, 0.0, 1.0;
        V << c * dt_, 0.0,
            s * dt_, 0.0,
            0.0, 0.0;
        robotstate_.mu = robotstate_.mu + Eigen::Vector3d{v * c * dt_, v * s * dt_, 0.0};
        // cout << robotstate_.mu << endl;
    } else {
        G << 1.0, 0.0, (v * (-c + c_dt)) / w,
            0.0, 1.0, (v * (-s + s_dt)) / w,
            0.0, 0.0, 1.0;
        V << (-s + s_dt) / w, v * (s - s_dt) / pow(w, 2) + v * dt_ * (c_dt) / w,
            (c - c_dt) / w, -v * (c - c_dt) / pow(w, 2) + v * dt_ * (s_dt) / w,
            0.0, dt_;
        robotstate_.mu = robotstate_.mu + Eigen::Vector3d{v * (-s + s_dt) / w, v * (c - c_dt) / w, w * dt_};
        robotstate_.mu(2) = angleLimitChecking(robotstate_.mu(2));
    }

    M << pow(p_a1_ * v, 2) + pow(p_a2_ * w, 2), 0.0,
        0.0, pow(p_a3_ * v, 2) + pow(p_a4_ * w, 2);
    robotstate_.sigma = G * robotstate_.sigma * G.transpose() + V * M * V.transpose();
}

void Ekf::predict_omni(double v, double w) {
    // TODO ekf predict function for omni
}

void Ekf::update() {
    // ekf update step:
    if (if_new_obstacles_) {
        // cout << "beacon: " << endl;
        // vector of the max j_max for each beacon
        vector<double> j_beacon_max = {p_mini_likelihood_, p_mini_likelihood_, p_mini_likelihood_};
        vector<Eigen::Vector3d> z_j_beacon_max = {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)};
        vector<Eigen::Vector3d> z_i_beacon_max = {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)};
        // cout << z_j_beacon_max[0] << z_j_beacon_max[1] << z_j_beacon_max[2] << endl;
        Eigen::Matrix3d zero_mat = Eigen::Matrix3d(Eigen::Vector3d{0.0, 0.0, 0.0}.asDiagonal());
        vector<Eigen::Matrix3d> H_j_beacon_max = {zero_mat, zero_mat, zero_mat};
        vector<Eigen::Matrix3d> S_j_beacon_max = {zero_mat, zero_mat, zero_mat};

        while (!beacon_from_scan_.empty()) {  // for all scanned beacon
            Eigen::Vector2d landmark_scan = beacon_from_scan_.back();
            beacon_from_scan_.pop_back();
            // transfer landmark_scan type from (x, y) to (r, phi)
            Eigen::Vector3d z_i = cartesianToPolar(landmark_scan, Eigen::Vector3d(0, 0, 0));
            // cout << "z_i: " << z_i << endl;
            double j_max = p_mini_likelihood_;
            int k_max = 0;
            int k = 0;
            Eigen::Vector3d z_j_max;
            Eigen::Matrix3d H_j_max;
            Eigen::Matrix3d S_j_max;

            // cout << "beacon: " << endl;

            for (Eigen::Vector2d landmark_k : beacon_in_map_) {  // for all beacon in map
                double j_k;
                Eigen::Vector3d z_k;
                Eigen::Matrix3d H_k;
                Eigen::Matrix3d S_k;
                std::tie(z_k, H_k) = cartesianToPolarWithH(landmark_k, robotstate_.mu);
                S_k = H_k * robotstate_.sigma * H_k.transpose() + Eigen::Matrix3d(Q_);
                try {
                    // original
                    // j_k = 1/sqrt((2*M_PI*S_k).determinant()) * exp(-0.5*(safeMinusTheta(z_i, z_k)).transpose()*S_k.inverse()*(safeMinusTheta(z_i, z_k)));
                    // ln(j_k()) version
                    j_k = -0.5 * safeMinusTheta(z_i, z_k).transpose() * S_k.inverse() * (safeMinusTheta(z_i, z_k)) - log(sqrt((2 * M_PI * S_k).determinant()));
                    // cout << j_k << endl;
                    if (j_k > j_max) {
                        j_max = j_k;
                        k_max = k;
                        z_j_max = z_k;
                        H_j_max = H_k;
                        S_j_max = S_k;
                    }
                } catch (const std::exception& e) {
                    ROS_ERROR("%s", e.what());
                    // std::cerr << e.what() << '\n';
                }
                k += 1;
            }
            // if(j_max > mini_likelihood_update_){
            //     Eigen::Matrix3d K_i;
            //     K_i = robotstate_.sigma*H_j_max.transpose()*S_j_max.inverse();
            //     robotstate_.mu += K_i*(z_i-z_j_max);
            //     robotstate_.sigma = (Eigen::Matrix3d::Identity() - K_i*H_j_max)*robotstate_.sigma;
            // }

            // only update three time but now it might update wrong beacon pillar
            // for the 3 beacon pillars, take out the largest 3 j value and z, H, S matrix
            if (j_max > j_beacon_max[k_max]) {
                j_beacon_max[k_max] = j_max;
                z_j_beacon_max[k_max] = z_j_max;
                z_i_beacon_max[k_max] = z_i;
                H_j_beacon_max[k_max] = H_j_max;
                S_j_beacon_max[k_max] = S_j_max;
                // for debug
                update_beacon_[k_max] = landmark_scan;
            }
        }
        for (int i = 0; i < 3; i++) {
            if (j_beacon_max[i] > p_mini_likelihood_update_) {
                // cout << "update, j = " << j_beacon_max[i] << endl;
                Eigen::Matrix3d K_i;
                K_i = robotstate_.sigma * H_j_beacon_max[i].transpose() * S_j_beacon_max[i].inverse();
                robotstate_.mu += K_i * (safeMinusTheta(z_i_beacon_max[i], z_j_beacon_max[i]));
                robotstate_.sigma = (Eigen::Matrix3d::Identity() - K_i * H_j_beacon_max[i]) * robotstate_.sigma;
            } else {
                update_beacon_[i] = Eigen::Vector2d(-1, -1);
                // cout << "didn't update, j = " << j_beacon_max[i] << endl;
            }
        }
    }
    // finish once ekf, change the flag
    if_new_obstacles_ = false;
}

double Ekf::euclideanDistance(Eigen::Vector2d a, Eigen::Vector3d b) {
    return sqrt(pow((b(0) - a(0)), 2) + pow((b(1) - a(1)), 2));
}

double Ekf::euclideanDistance(Eigen::Vector2d a, Eigen::Vector2d b) {
    return sqrt(pow((b(0) - a(0)), 2) + pow((b(1) - a(1)), 2));
}

double Ekf::angleLimitChecking(double theta) {
    while (theta > M_PI) {
        theta -= M_PI * 2;
    }
    while (theta <= -M_PI) {
        theta += M_PI * 2;
    }
    return theta;
}

Eigen::Vector3d Ekf::safeMinusTheta(Eigen::Vector3d a, Eigen::Vector3d b) {
    // in this function, minus z(r,theta,signature) and check z(1), theta, is in the bound
    Eigen::Vector3d a_minus_b = a - b;
    a_minus_b(1) = angleLimitChecking(a_minus_b(1));
    return a_minus_b;
}

Eigen::Vector3d Ekf::cartesianToPolar(Eigen::Vector2d point, Eigen::Vector3d origin) {
    // transpose point from cartesian to polar with given origin
    double q_sqrt = euclideanDistance(point, origin);
    double q = pow(q_sqrt, 2);
    double dx = point(0) - origin(0);
    double dy = point(1) - origin(1);
    Eigen::Vector3d z;
    z << q_sqrt, atan2(dy, dx) - origin(2), 1.0;
    z(1) = angleLimitChecking(z(1));
    return z;
}

std::tuple<Eigen::Vector3d, Eigen::Matrix3d> Ekf::cartesianToPolarWithH(Eigen::Vector2d point, Eigen::Vector3d origin) {
    // transpose point from cartesian to polar with given origin
    double q_sqrt = euclideanDistance(point, origin);
    double q = pow(q_sqrt, 2);
    double dx = point(0) - origin(0);
    double dy = point(1) - origin(1);
    Eigen::Vector3d z;
    Eigen::Matrix3d H;
    z << q_sqrt, atan2(dy, dx) - origin(2), 1.0;
    z(1) = angleLimitChecking(z(1));
    H << -(dx / q_sqrt), -(dy / q_sqrt), 0,
        dy / q, -dx / q, -1,
        0, 0, 0;
    return std::make_tuple(z, H);
}

Eigen::Vector2d Ekf::tfBasefpToMap(Eigen::Vector2d point, Eigen::Vector3d robot_pose) {
    double s = sin(robot_pose(2));
    double c = cos(robot_pose(2));
    Eigen::Matrix2d rotation_mat;
    rotation_mat << c, -s, s, c;
    Eigen::Vector2d point_map = rotation_mat * point + Eigen::Vector2d{robot_pose(0), robot_pose(1)};
    return point_map;
}

void Ekf::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    const ros::Time stamp = ros::Time::now() + ros::Duration(0.2);
    double v = odom_msg->twist.twist.linear.x;
    double w = odom_msg->twist.twist.angular.z;
    dt_ = odom_msg->header.stamp.toSec() - last_cb_time_.toSec();
    last_cb_time_ = odom_msg->header.stamp;
    // cout << "v: " << v << "w: " << w << endl;
    // for calculate time cost
    // struct timespec tt1, tt2;
    // clock_gettime(CLOCK_REALTIME, &tt1);

    predict_diff(v, w);
    update();

    // clock_gettime(CLOCK_REALTIME, &tt2);
    // count_ += 1;
    // duration_ += (tt2.tv_nsec-tt1.tv_nsec)*1e-9;
    // cout << "average time cost is " << duration_/count_ << "s" << endl;

    publishEkfPose(stamp);  // stamp = acturally when does tf been generated
    publishUpdateBeacon(stamp);
    broadcastEkfTransform(odom_msg);  // stamp = odom.stamp so frequency = odom's frequency
}

void Ekf::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg) {
    if_new_obstacles_ = false;
    beacon_from_scan_.clear();  // drop out all element in list
    // cout << "obstacle to map frame is: " << endl;
    for (int i = 0; i < obstacle_msg->circles.size(); i++) {
        obstacle_detector::CircleObstacle circle = obstacle_msg->circles[i];
        Eigen::Vector2d xy(circle.center.x, circle.center.y);
        Eigen::Vector2d xy_map = tfBasefpToMap(xy, robotstate_.mu);
        // filter out those obstacles radius bigger than 0.1m
        // if(circle.true_radius >= 0.1){
        //     continue;
        // }
        // filter out those obstacles position do not close the beacon pillar on map
        for (auto const& i : beacon_in_map_) {
            double distance = euclideanDistance(xy_map, i);
            if (distance < p_max_obstacle_distance_) {  // alst time in real world experience we take < 0.5
                // cout << xy_map << endl;
                beacon_from_scan_.push_back(xy);
            }
        }
        // cout << xy << endl;
        // beacon_from_scan_.push_back(xy);
    }
    if_new_obstacles_ = true;
}

void Ekf::poseEstimateCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg) {
    robotstate_.mu << pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, tf2::getYaw(pose_msg->pose.pose.orientation);
    robotstate_.sigma << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    std::string print_robot_name = (p_robot_name_ == "") ? "robot" : p_robot_name_;
    ROS_INFO("the %s pose is set to: (%.2f, %.2f, %.2f)", print_robot_name.c_str(), robotstate_.mu(0), robotstate_.mu(1), robotstate_.mu(2));
}

bool Ekf::resetEkf(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    robotstate_.mu << p_initial_x_, p_initial_y_, degToRad(p_initial_theta_deg_);
    robotstate_.sigma << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    std::string print_robot_name = (p_robot_name_ == "") ? "robot" : p_robot_name_;
    ROS_INFO("the %s pose is set to: (%.2f, %.2f, %.2f)", print_robot_name.c_str(), robotstate_.mu(0), robotstate_.mu(1), robotstate_.mu(2));
    return true;
}

void Ekf::publishEkfPose(const ros::Time& stamp) {
    geometry_msgs::PoseWithCovarianceStamped ekf_pose;
    ekf_pose.header.stamp = stamp;
    ekf_pose.header.frame_id = "map";
    ekf_pose.pose.pose.position.x = robotstate_.mu(0);
    ekf_pose.pose.pose.position.y = robotstate_.mu(1);
    tf2::Quaternion q;
    q.setRPY(0, 0, robotstate_.mu(2));
    ekf_pose.pose.pose.orientation.x = q.x();
    ekf_pose.pose.pose.orientation.y = q.y();
    ekf_pose.pose.pose.orientation.z = q.z();
    ekf_pose.pose.pose.orientation.w = q.w();
    ekf_pose.pose.covariance[0] = robotstate_.sigma(0, 0);   // x-x
    ekf_pose.pose.covariance[1] = robotstate_.sigma(0, 1);   // x-y
    ekf_pose.pose.covariance[5] = robotstate_.sigma(0, 2);   // x-theta
    ekf_pose.pose.covariance[6] = robotstate_.sigma(1, 0);   // y-x
    ekf_pose.pose.covariance[7] = robotstate_.sigma(1, 1);   // y-y
    ekf_pose.pose.covariance[11] = robotstate_.sigma(1, 2);  // y-theta
    ekf_pose.pose.covariance[30] = robotstate_.sigma(2, 0);  // theta-x
    ekf_pose.pose.covariance[31] = robotstate_.sigma(2, 1);  // theta-y
    ekf_pose.pose.covariance[35] = robotstate_.sigma(2, 2);  // theta-theta
    ekf_pose_pub_.publish(ekf_pose);
}
void Ekf::broadcastEkfTransform(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    tf2::Transform map_to_baseft(
        tf2::Quaternion(tf2::Vector3(0, 0, 1), robotstate_.mu(2)),
        tf2::Vector3(robotstate_.mu(0), robotstate_.mu(1), 0));
    tf2::Transform odom_to_baseft(
        tf2::Quaternion(0, 0, odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w),
        tf2::Vector3(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, 0));
    tf2::Transform map_to_odom = map_to_baseft * odom_to_baseft.inverse();

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = odom_msg->header.stamp + ros::Duration(0.1);
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = p_robot_name_ + "/odom";
    transformStamped.transform = tf2::toMsg(map_to_odom);
    br_.sendTransform(transformStamped);
}

void Ekf::publishUpdateBeacon(const ros::Time& stamp) {
    obstacle_detector::Obstacles update_obstacles;
    for (Eigen::Vector2d o : update_beacon_) {
        if (o(0) == -1) continue;
        obstacle_detector::CircleObstacle circle;
        circle.center.x = o(0);
        circle.center.y = o(1);
        circle.radius = 0.35;
        circle.true_radius = 0.05;
        update_obstacles.circles.push_back(circle);
    }
    if (p_robot_name_ == "")
        update_obstacles.header.frame_id = "base_footprint";
    else
        update_obstacles.header.frame_id = p_robot_name_ + "/base_footprint";
    update_beacon_pub_.publish(update_obstacles);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ekf_localization");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    Ekf ekf(nh, nh_local);
    ekf.initialize();

    while (ros::ok()) {
        ros::spin();
    }
}