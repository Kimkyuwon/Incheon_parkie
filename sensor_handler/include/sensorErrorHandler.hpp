#ifndef SENSOR_ERROR_HANDLER_HPP_
#define SENSOR_ERROR_HANDLER_HPP_
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
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>

class sensorErrorHandler
    : public rclcpp::Node
{
public:
    sensorErrorHandler();
    ~sensorErrorHandler();
    int Initialize();
    void checkError();

    void CntHandler(const nav_msgs::msg::Odometry::SharedPtr cnt);
private:

    rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr subCnt;

    int imu_cnt, odom_cnt, gnss_cnt, lidar_cnt;
    int imu_flag, odom_flag, gnss_flag, lidar_flag;
    int lidar_delay_cnt;
    bool cnt_subFlag = false;

    rclcpp::QoS qos_;
    rclcpp::TimerBase::SharedPtr timer1hz;
};
#endif // SENSOR_ERROR_HANDLER_HPP_
