#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include "common.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2/transform_datatypes.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/impl/utils.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"

using namespace std;

class DR_Handler
    : public rclcpp::Node{
public:
    DR_Handler();
    ~DR_Handler();
    int Initialize();
    void pub();

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg);
    void odomHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg);

private:
    int bias_cnt = 0;
    double roll_bias = 0;
    double pitch_bias = 0;
    double yaw_bias = 0;
    bool odom_received = false;

    double dt_imu, prev_time_imu;
    double accel_x, accel_y;
    double velo_x, velo_y, velo_x_temp, velo_y_temp;

    Eigen::Vector3d gyro;
    Eigen::Vector3d acc;
    Eigen::Vector3d odom_vec;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubDR;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubDummyImu;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubDummyOdom;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr subIMU;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr subOdom;

    rclcpp::QoS qos_;
    rclcpp::TimerBase::SharedPtr timer;
};
