#ifndef POSE_ESTIMATION__POSE_ESTIMATION_HPP_
#define POSE_ESTIMATION__POSE_ESTIMATION_HPP_
#include <fstream>
#include <math.h>
#include <vector>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>
#include <filesystem>
#include <mutex>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <std_msgs/msg/bool.hpp>
#include "std_msgs/msg/empty.hpp"
#include <geometry_msgs/msg/twist_stamped.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/float32.hpp>
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include <tf2/transform_datatypes.h>
#include "tf2_ros/transform_broadcaster.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include "tf2/transform_storage.h"
#include "tf2/utils.h"

#include "common.h"
#include "tic_toc.h"

using namespace std;
namespace pose_estimation{
class poseEstimation
    : public rclcpp::Node
{
public:
    poseEstimation();
    ~poseEstimation();

    int initialize();
    void poseEstimate();

    void initPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr init);
    void lidarPoseCallback(const nav_msgs::msg::Path::SharedPtr meas);
    void gnssPoseCallback(const nav_msgs::msg::Odometry::SharedPtr gnssPose);
    void drVeloCallback(const nav_msgs::msg::Odometry::SharedPtr drVelo);
    void flagCallback(const std_msgs::msg::Empty::SharedPtr flag);    
    void responseCallback(const std_msgs::msg::Bool::SharedPtr rspsMsg);


private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubRobotPose;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubRobotBodyPose;

    // add start
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubEvaluationRobotBodyPose;
    // add end

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pubReadyFlag;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubResponse;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr subDR;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr subGNSS;
    rclcpp::Subscription<nav_msgs::msg::Path>::ConstSharedPtr subMeas;
    rclcpp::Subscription<std_msgs::msg::Empty>::ConstSharedPtr subReadyFlag;
    rclcpp::Subscription<std_msgs::msg::Bool>::ConstSharedPtr subResponse;
    
    Eigen::MatrixXd g_mat_Q, g_mat_H, g_mat_P, g_mat_R, g_mat_K, g_mat_I;

    Eigen::VectorXd g_state_X;
    Eigen::VectorXd g_state_X_pre;
    Eigen::VectorXd g_res_Z;

    Eigen::VectorXd g_X_gnss;
    Eigen::VectorXd g_X_gnss_pre;
    Eigen::VectorXd g_X_lidar;
    Eigen::VectorXd g_X_lidar_ori;

    Eigen::VectorXd g_Q_var;
    Eigen::VectorXd g_R_lidar;
    Eigen::VectorXd g_R_GNSS;

    Eigen::VectorXd g_TM_offset;

    Eigen::Vector3d velocity;
    Eigen::Vector3d rpy_inc;

    bool lidar_pose_received = false;
    bool gnss_pose_received = false;
    bool gnss_handler_on = false;
    bool state_init =false;
    bool flag_published = false;
    bool response_subscribed = false;

    double g_dt, prev_time;
    double lidar_prev_time;
    double init_x, init_y, init_z, init_roll, init_pitch, init_yaw;
    double std_gnss_x, std_gnss_y, std_gnss_z;
    double std_gnss_roll, std_gnss_pitch, std_gnss_yaw;
    int pos_type;
    int num_sat;
    bool init_gnss = 0;
    int init_cnt = 0;
    int timer_cnt = 0;
    int gnss_cnt = 0;
    int measure_update_type = 0;
    double gnss_time;
    double lidar_time;
    
    rclcpp::QoS qos_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    geometry_msgs::msg::TransformStamped tf;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster2;
    geometry_msgs::msg::TransformStamped tf2;

    // add start
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_evaluation;
    geometry_msgs::msg::TransformStamped tf_evaluation;
    double LaserTrackerOffset_x;
    double LaserTrackerOffset_y;
    // add end

    geometry_msgs::msg::PoseStamped meas_info;
    geometry_msgs::msg::PoseStamped hessian_cov;

    std::mutex mtx;

    rclcpp::TimerBase::SharedPtr timer;
};
}
#endif // POSE_ESTIMATION__POSE_ESTIMATION_HPP_
