#include <ros/ros.h>
// tf2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
// for matrix calculate
#include <Eigen/Dense>
#include <math.h>
// msg
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include "obstacle_detector/Obstacles.h"

class Ekf{
    public:
        Ekf(ros::NodeHandle& nh);
    private:
        void initialize();
        // ekf 
        void predict();
        void update();
        // several util function
        double euclideanDistance();
        double angleLimitChecking();
        void cartesianToPolar();
        double degToRad(double deg){ return deg*M_PI/180.0 ;}
        // for ros
        void odomCallback(const nav_msgs::Odometry::ConstPtr odom_msg);
        void obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr obstacle_msg);
        void publishEkfPose();
        void broadcastEkfTransform();

        // for beacon position in map std::list<double>{ax, ay, bx, by, cx, cy}
        std::list<Eigen::Vector2d> beacon_in_map_;

        // for beacon piller detection
        bool if_new_obstacles_;
        std::list<Eigen::Vector2d> beacon_from_scan_;

        // for robot state
        Eigen::Vector3d mu_0_;
        Eigen::Vector3d mu_past_;
        Eigen::Vector3d mu_;
        Eigen::Matrix3d sigma_past_;
        Eigen::Matrix3d sigma_;
        double dt_;

        // ros parameter
        std::string p_robot_name_;
        double p_initial_x_;
        double p_initial_y_;
        double p_initial_theta_deg_;
        
        // ros node
        ros::NodeHandle nh_;
        ros::Subscriber odom_sub_;
        ros::Subscriber raw_obstacles_sub_;
        ros::Publisher ekf_pose_pub_;
        tf2_ros::TransformBroadcaster br_;

};
