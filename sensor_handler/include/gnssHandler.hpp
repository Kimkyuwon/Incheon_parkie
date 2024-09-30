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
#include <novatel_oem7_msgs/msg/inspvax.hpp>
#include <novatel_oem7_msgs/msg/insstdev.hpp>
#include <novatel_oem7_msgs/msg/bestpos.hpp>

using namespace std;

class gnssHandler
    : public rclcpp::Node{
public:
    gnssHandler();
    ~gnssHandler();
    int Initialize();

    void callbackGnss(const novatel_oem7_msgs::msg::INSPVAX::SharedPtr gnssMsg);
    void callbackInsStdDev(const novatel_oem7_msgs::msg::INSSTDEV::SharedPtr stdMsg);
    void callbackBestPos(const novatel_oem7_msgs::msg::BESTPOS::SharedPtr posMsg);

    void responseCallback(const std_msgs::msg::Bool::SharedPtr rspsMsg);

private:
    double timeDr = 0.0;
    double timeDrNano = 0.0;
    bool flag_published = false;
    bool response_subscribed = false;
    double lat_std, lon_std, hgt_std, N_vel_std, E_vel_std, U_vel_std, roll_std, pitch_std, azi_std;
    int num_sv;
    int pos_type;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pubReadyFlag;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubGnssPose;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubDummyGnss;
    rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVAX>::ConstSharedPtr subGNSS;
    rclcpp::Subscription<novatel_oem7_msgs::msg::INSSTDEV>::ConstSharedPtr subINSSTDEV;
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTPOS>::ConstSharedPtr subBestPos;
    rclcpp::Subscription<std_msgs::msg::Bool>::ConstSharedPtr subResponse;

    rclcpp::QoS qos_;
    rclcpp::TimerBase::SharedPtr timer;
};
