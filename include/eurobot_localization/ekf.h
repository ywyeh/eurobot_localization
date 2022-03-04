#include <ros/ros.h>
#include <ros/console.h>
#include <time.h>
// tf2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// for matrix calculate
#include <Eigen/Dense>
#include <math.h>
// msg
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include "obstacle_detector/Obstacles.h"

struct RobotState{
    Eigen::Vector3d mu;
    Eigen::Matrix3d sigma;
};

class Ekf{
    public:
        Ekf(ros::NodeHandle& nh): nh_(nh){};
        void initialize();
    private:
        // ekf 
        void predict_diff(double v, double w);
        void predict_omni(double v, double w);
        void update();

        // several util function
        double euclideanDistance(Eigen::Vector2d a, Eigen::Vector3d b);
        double euclideanDistance(Eigen::Vector2d a, Eigen::Vector2d b);
        double angleLimitChecking(double theta);
        Eigen::Vector3d safeMinusTheta(Eigen::Vector3d a, Eigen::Vector3d b);
        Eigen::Vector3d cartesianToPolar(Eigen::Vector2d point, Eigen::Vector3d origin);
        std::tuple<Eigen::Vector3d, Eigen::Matrix3d> cartesianToPolarWithH(Eigen::Vector2d point, Eigen::Vector3d origin);
        Eigen::Vector2d tfBasefpToMap(Eigen::Vector2d point, Eigen::Vector3d robot_pose);
        double degToRad(double deg){ return deg*M_PI/180.0 ;}
        
        // for ros
        void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
        void obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg);
        void publishEkfPose(const ros::Time& stamp);
        void publishUpdateBeacon(const ros::Time& stamp);
        void broadcastEkfTransform(const nav_msgs::Odometry::ConstPtr& odom_msg);

        // for beacon position in map std::list<double>{ax, ay, bx, by, cx, cy}
        double p_beacon_ax_;
        double p_beacon_ay_;
        double p_beacon_bx_;
        double p_beacon_by_;
        double p_beacon_cx_;
        double p_beacon_cy_;
        std::list<Eigen::Vector2d> beacon_in_map_;

        // for beacon piller detection
        bool if_new_obstacles_;
        std::list<Eigen::Vector2d> beacon_from_scan_;

        // for debug
        std::vector<Eigen::Vector2d> update_beacon_;

        // for robot state
        Eigen::Vector3d mu_0_;
        RobotState robotstate_past_;
        RobotState robotstate_bar_;
        RobotState robotstate_;
        double p_odom_freq_;
        double dt_;

        // ekf parameter
        // motion covariance
        double p_a1_;
        double p_a2_;
        double p_a3_;
        double p_a4_;

        // measure noise
        double p_Q1_;
        double p_Q2_;
        double p_Q3_;
        Eigen::DiagonalMatrix<double, 3> Q_;

        // set minimum likelihood value
        double p_mini_likelihood_;
        double p_mini_likelihood_update_;
        double p_max_obstacle_distance_;

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
        // for debug
        ros::Publisher update_beacon_pub_;

        // for function time calculation
        int count_;
        double duration_;
};
