#ifndef SENSOR_CHECKER_HPP_
#define SENSOR_CHECKER_HPP_
#include <fstream>
#include <math.h>
#include <vector>
#include <queue>
#include <thread>
#include <iostream>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist_stamped.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/float32.hpp>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>

// #include <ublox_gps/gps.hpp>
#include <ublox_msgs/ublox_msgs.hpp>

class sensorChecker
    : public rclcpp::Node
{
public:
    sensorChecker();
    ~sensorChecker();
    int Initialize();
    void pub1hz();

    void imuHandler(const std_msgs::msg::Float32::SharedPtr imu);
    void odomHandler(const std_msgs::msg::Float32::SharedPtr odom);
    void gnssHandler (const std_msgs::msg::Float32::SharedPtr gnss);
    void lidarHandler(const std_msgs::msg::Float32::SharedPtr lidar);

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::ConstSharedPtr subImu;
    rclcpp::Subscription<std_msgs::msg::Float32>::ConstSharedPtr subOdom;
    rclcpp::Subscription<std_msgs::msg::Float32>::ConstSharedPtr subGnss;
    rclcpp::Subscription<std_msgs::msg::Float32>::ConstSharedPtr subLidar;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubCnt;

    int imu_cnt, odom_cnt, gnss_cnt, lidar_cnt;
    bool imu_flag = false;
    bool odom_flag = false;
    bool gnss_flag = false;
    bool lidar_flag = false;

    rclcpp::QoS qos_;
    rclcpp::TimerBase::SharedPtr timer1hz;
};
#endif // SENSOR_CHECKER_HPP_
