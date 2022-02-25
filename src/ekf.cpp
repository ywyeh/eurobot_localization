#include "eurobot_localization/ekf.h"
using namespace std;

void Ekf::initialize(){
    // get parameter
    nh_.param<string>("robot_name", p_robot_name_, "");
    nh_.param<double>("initial_x", p_initial_x_, 0.5);
    nh_.param<double>("initial_y", p_initial_y_, 0.5);
    nh_.param<double>("initial_theta", p_initial_theta_deg_, 90);

    // for beacon position in map list<double>{ax, ay, bx, by, cx, cy}
    Eigen::Vector2d beacon_a {0.05,  3.1};
    Eigen::Vector2d beacon_b {1.05, -0.1};
    Eigen::Vector2d beacon_c {1.95,  3.1};
    beacon_in_map_ = {beacon_a, beacon_b, beacon_c};
    // for (Eigen::Vector2d n : beacon_in_map_){
    //     cout << n;
    // }
    // for robot state
    mu_0_ << p_initial_x_, p_initial_y_, degToRad(p_initial_theta_deg_);
    robotstate_past_.mu = mu_0_;
    robotstate_past_.sigma << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    robotstate_bar_.mu << 0, 0, 0;
    robotstate_bar_.sigma << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    robotstate_.mu << 0, 0, 0;
    robotstate_.sigma << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    dt_ = 1/50;

    // for beacon piller detection
    if_new_obstacles_ = false;
    beacon_from_scan_ = {};

    // for ros
    odom_sub_ = nh_.subscribe(p_robot_name_+"odom", 50, &Ekf::odomCallback, this);
    raw_obstacles_sub_ = nh_.subscribe(p_robot_name_+"raw_obstacles", 10, &Ekf::obstaclesCallback, this);
    ekf_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(p_robot_name_+"ekf_pose", 10);  
}

void Ekf::predict_diff(double v, double w){
    // cout << v << w << endl;
}

void Ekf::predict_ormi(double v, double w){
    // TODO ekf predict function for ormi
}

void Ekf::update(){

robotstate_ = robotstate_past_; // for tf testing


}

double Ekf::euclideanDistance(Eigen::Vector3d a, Eigen::Vector3d b){
    return sqrt(pow((b[0]-a[0]), 2) + pow((b[1]-a[1]), 2));
}
double Ekf::angleLimitChecking(double theta){
    if(theta > M_PI){
        theta -= M_PI*2;
    }
    else if (theta <= -M_PI){
        theta += M_PI*2;
    }
    return theta;
}
void Ekf::cartesianToPolar(){

}

void Ekf::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
    const ros::Time stamp = ros::Time::now() + ros::Duration(0.2);
    double v = odom_msg->twist.twist.linear.x;
    double w = odom_msg->twist.twist.angular.z;
    predict_diff(v, w);
    update();
    publishEkfPose(stamp); // stamp = acturally when does tf been generated
    broadcastEkfTransform(odom_msg); // stamp = odom.stamp so frequency = odom's frequency
}

void Ekf::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg){
    if_new_obstacles_ = false;
    beacon_from_scan_.clear(); // drop out all element in list
    for(int i = 0; i < obstacle_msg->circles.size(); i++){
        obstacle_detector::CircleObstacle circle = obstacle_msg->circles[i];
        // TODO filter out those obstacles radius do not meet the beacon pillar
        Eigen::Vector2d xy(circle.center.x, circle.center.y);
        beacon_from_scan_.push_back(xy);
    }
    if_new_obstacles_ = true;
}

void Ekf::publishEkfPose(const ros::Time& stamp){
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
    ekf_pose.pose.covariance[0] = robotstate_.sigma(0,0); // x-x
    ekf_pose.pose.covariance[1] = robotstate_.sigma(0,1); // x-y
    ekf_pose.pose.covariance[5] = robotstate_.sigma(0,2); // x-theta
    ekf_pose.pose.covariance[6] = robotstate_.sigma(1,0); // y-x
    ekf_pose.pose.covariance[7] = robotstate_.sigma(1,1); // y-y
    ekf_pose.pose.covariance[11] = robotstate_.sigma(1,2); // y-theta
    ekf_pose.pose.covariance[30] = robotstate_.sigma(2,0); // theta-x
    ekf_pose.pose.covariance[31] = robotstate_.sigma(2,1); // theta-y
    ekf_pose.pose.covariance[35] = robotstate_.sigma(2,2); // theta-theta
    ekf_pose_pub_.publish(ekf_pose);
}
void Ekf::broadcastEkfTransform(const nav_msgs::Odometry::ConstPtr& odom_msg){
    tf2::Transform map_to_baseft(
        tf2::Quaternion(tf2::Vector3(0, 0, 1), robotstate_.mu(2)),
        tf2::Vector3(robotstate_.mu(0), robotstate_.mu(1), 0)
    );
    tf2::Transform odom_to_baseft(
        tf2::Quaternion(0, 0, odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w),
        tf2::Vector3(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, 0)
    );
    tf2::Transform map_to_odom = odom_to_baseft.inverse()*map_to_baseft;

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = odom_msg->header.stamp;
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = p_robot_name_+"odom";
    transformStamped.transform = tf2::toMsg(map_to_odom);
    br_.sendTransform(transformStamped);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ekf_localization");
    ros::NodeHandle nh;
    Ekf ekf(nh);
    ekf.initialize();
    while(ros::ok()){
        ros::spin();
    }
}