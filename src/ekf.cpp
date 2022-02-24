#include "eurobot_localization/ekf.h"

Ekf::Ekf(ros::NodeHandle& nh): nh_(nh){
    initialize();
}

void Ekf::initialize(){
    // get parameter
    nh_.param<std::string>("robot_name", p_robot_name_, "robot1");
    nh_.param<double>("initial_x", p_initial_x_, 0.5);
    nh_.param<double>("initial_y", p_initial_y_, 0.5);
    nh_.param<double>("initial_theta", p_initial_theta_deg_, 90);

    // for beacon position in map std::list<double>{ax, ay, bx, by, cx, cy}
    Eigen::Vector2d beacon_a {0.05,  3.1};
    Eigen::Vector2d beacon_b {1.05, -0.1};
    Eigen::Vector2d beacon_c {1.95,  3.1};
    beacon_in_map_ = {beacon_a, beacon_b, beacon_c};
    // for (Eigen::Vector2d n : beacon_in_map_){
    //     std::cout << n;
    // }
    // for robot state
    mu_0_ << p_initial_x_, p_initial_y_, degToRad(p_initial_theta_deg_);
    mu_past_ = mu_0_;
    mu_ << 0, 0, 0;
    sigma_past_ << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    sigma_ << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    dt_ = 1/50;

    // for beacon piller detection
    if_new_obstacles_ = false;
    beacon_from_scan_ = {};

    // for ros
    odom_sub_ = nh_.subscribe(p_robot_name_+"odom", 50, &Ekf::odomCallback, this);
    raw_obstacles_sub_ = nh_.subscribe(p_robot_name_+"raw_obstacles", 10, &Ekf::obstaclesCallback, this);
    ekf_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(p_robot_name_+"ekf_pose", 10);
}

void Ekf::predict(){

}
void Ekf::update(){

}

double Ekf::euclideanDistance(){
    return 0;
}
double Ekf::angleLimitChecking(){
    return 0;
}
void Ekf::cartesianToPolar(){

}

void Ekf::odomCallback(const nav_msgs::Odometry::ConstPtr odom_msg){
    // ROS_INFO(odom_msg);
}

void Ekf::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr obstacle_msg){

}

void Ekf::publishEkfPose(){
    geometry_msgs::PoseWithCovarianceStamped pose;
}
void Ekf::broadcastEkfTransform(){

}

int main(int argc, char** argv){
    ros::init(argc, argv, "ekf_localization");
    ros::NodeHandle nh;

    Ekf ekf(nh);
}