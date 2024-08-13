#include "sensorErrorHandler.hpp"

using namespace std;

sensorErrorHandler::sensorErrorHandler()
    : rclcpp::Node("sensorErrorHandler")
    , qos_(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_services_default))
{
    Initialize();
}

sensorErrorHandler::~sensorErrorHandler(){}

int sensorErrorHandler::Initialize()
{
    lidar_delay_cnt = 0;

    subCnt = create_subscription<nav_msgs::msg::Odometry>(
        "sensor_cnt", qos_, std::bind(&sensorErrorHandler::CntHandler, this, std::placeholders::_1));

    timer1hz = this->create_wall_timer(std::chrono::milliseconds(1000),
                                    std::bind(&sensorErrorHandler::checkError, this));

    return 0;
}

void sensorErrorHandler::CntHandler(const nav_msgs::msg::Odometry::SharedPtr cnt)
{
    imu_cnt = cnt->pose.covariance[0];
    odom_cnt = cnt->pose.covariance[1];
    gnss_cnt = cnt->pose.covariance[2];
    lidar_cnt = cnt->pose.covariance[3];
    imu_flag = cnt->pose.covariance[4];
    odom_flag = cnt->pose.covariance[5];
    gnss_flag = cnt->pose.covariance[6];
    lidar_flag = cnt->pose.covariance[7];
    cnt_subFlag = true;
}

void sensorErrorHandler::checkError()
{
    if (!cnt_subFlag)
    {
        RCLCPP_INFO(this->get_logger(),"Sensor Checker is not working.");
        return;
    }

    if (imu_flag && imu_cnt < 80)
    {
        RCLCPP_INFO(this->get_logger(),"IMU data is unstable. (%d hz)", imu_cnt);   
    }
    if (odom_flag && odom_cnt < 80)
    {
        RCLCPP_INFO(this->get_logger(),"Odometer data is unstable. (%d hz)", odom_cnt);
    }
    else if (odom_cnt > 150)
    {
        RCLCPP_INFO(this->get_logger(),"Odometer data is unstable. (%d hz)", odom_cnt);
    }
    if (lidar_flag && lidar_cnt < 8)
    {
        RCLCPP_INFO(this->get_logger(),"LIDAR data is unstable. (%d hz)", lidar_cnt);
    }

    if (imu_flag == 0)
    {
        RCLCPP_INFO(this->get_logger(),"IMU data is not received.");
    }
    if (odom_flag == 0)
    {
        RCLCPP_INFO(this->get_logger(),"Odometer data is not received.");
    }
    if (gnss_flag == 0)
    {
        RCLCPP_INFO(this->get_logger(),"GNSS data is not received.");
    }
    if (lidar_flag == 0)
    {
        lidar_delay_cnt++;
    }
    else
    {
        lidar_delay_cnt = 0;
    }
    if (lidar_delay_cnt >= 5)
    {
        RCLCPP_INFO(this->get_logger(),"LiDAR data is not received. (%d sec)", lidar_delay_cnt);
    }
    cnt_subFlag = false;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<sensorErrorHandler>());

    return 0;
}
