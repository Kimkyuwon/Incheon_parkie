#include "DR_Handler.hpp"

DR_Handler::DR_Handler()
    : rclcpp::Node("DR_Handler")
    , qos_(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_services_default))
{
    Initialize();
}

DR_Handler::~DR_Handler()
{
}

int DR_Handler::Initialize()
{
    odom_vec.setZero();
    prev_time_imu = 0;

    subOdom = create_subscription<nav_msgs::msg::Odometry>(
        "/novatel/oem7/odom", qos_, std::bind(&DR_Handler::odomHandler, this, std::placeholders::_1)); // odom

    pubDR = create_publisher<nav_msgs::msg::Odometry>("dr/velo", qos_);
    pubDummyImu = create_publisher<std_msgs::msg::Float32>("dummyImu", qos_);
    pubDummyOdom = create_publisher<std_msgs::msg::Float32>("dummyOdom", qos_);

    return 0;
}

void DR_Handler::odomHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg){

    std_msgs::msg::Float32 dummyMsg;
    pubDummyOdom->publish(dummyMsg);
    
    odom_received = true;
    
    /* linear velocity from Odom, angular velocity from IMU, linear acceleration from IMU(just store at covariance) */
    nav_msgs::msg::Odometry drVelo;
    drVelo.header.stamp = odomMsg->header.stamp;
    drVelo.twist.twist.linear.x = odomMsg->twist.twist.linear.y; //
    drVelo.twist.twist.linear.y = odomMsg->twist.twist.linear.x; //
    drVelo.twist.twist.linear.z = odomMsg->twist.twist.linear.z; //

    drVelo.twist.twist.angular.x = rad2deg(odomMsg->twist.twist.angular.z); //roll 각속도
    drVelo.twist.twist.angular.y = -rad2deg(odomMsg->twist.twist.angular.x); //pitch 각속도
    drVelo.twist.twist.angular.z = rad2deg(odomMsg->twist.twist.angular.z); //yaw 각속도

    pubDR->publish(drVelo);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<DR_Handler>());

    return 0;
}
