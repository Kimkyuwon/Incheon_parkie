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
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include <novatel_oem7_msgs/msg/inspva.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>  //Add LimJH

using namespace std;

class gnssHandler
    : public rclcpp::Node{
public:
    gnssHandler();
    ~gnssHandler();
    int Initialize();

    void callbackGnss(const novatel_oem7_msgs::msg::INSPVA::SharedPtr gnssMsg);
    // void LatLon2TM(double lat, double lon, double &x, double &y);

    void callbackNMEA(const sensor_msgs::msg::NavSatFix::SharedPtr nmeaMsg);    //Add LimJH
    void responseCallback(const std_msgs::msg::Bool::SharedPtr rspsMsg);

private:
    double timeDr = 0.0;
    double timeDrNano = 0.0;
    bool flag_published = false;
    bool response_subscribed = false;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pubReadyFlag;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubGnssPose;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubDummyGnss;
    rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVA>::ConstSharedPtr subGNSS;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::ConstSharedPtr subNMEA;  //Add LimJH
    rclcpp::Subscription<std_msgs::msg::Bool>::ConstSharedPtr subResponse;

    rclcpp::QoS qos_;
    rclcpp::TimerBase::SharedPtr timer;
};
