#include "sensorChecker.hpp"

using namespace std;

sensorChecker::sensorChecker()
    : rclcpp::Node("sensorChecker")
    , qos_(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_services_default))
{
    Initialize();
}

sensorChecker::~sensorChecker(){}

int sensorChecker::Initialize(){

    subImu = create_subscription<std_msgs::msg::Float32>(
        "dummyImu", qos_, std::bind(&sensorChecker::imuHandler, this, std::placeholders::_1));
    subOdom = create_subscription<std_msgs::msg::Float32>(
        "dummyOdom", qos_, std::bind(&sensorChecker::odomHandler, this, std::placeholders::_1));
    subGnss = create_subscription<std_msgs::msg::Float32>(
        "dummyGnss", qos_, std::bind(&sensorChecker::gnssHandler, this, std::placeholders::_1));
    subLidar = create_subscription<std_msgs::msg::Float32>(
        "dummyLidar", qos_, std::bind(&sensorChecker::lidarHandler, this, std::placeholders::_1));

    pubCnt = create_publisher<nav_msgs::msg::Odometry>("sensor_cnt", qos_);

    timer1hz = this->create_wall_timer(std::chrono::milliseconds(1000),
                                     std::bind(&sensorChecker::pub1hz, this));

    return 0;
}

void sensorChecker::imuHandler(const std_msgs::msg::Float32::SharedPtr imu)
{
    imu_cnt++;
}

void sensorChecker::odomHandler(const std_msgs::msg::Float32::SharedPtr odom)
{
    odom_cnt++;
}

void sensorChecker::gnssHandler(const std_msgs::msg::Float32::SharedPtr gnss)
{
    gnss_cnt++;
}

void sensorChecker::lidarHandler(const std_msgs::msg::Float32::SharedPtr lidar)
{
    lidar_cnt++;
}

void sensorChecker::pub1hz()
{
    if (imu_cnt > 0)  imu_flag = true;
    if (odom_cnt > 0)  odom_flag = true;
    if (gnss_cnt > 0)  gnss_flag = true;
    if (lidar_cnt > 0)  lidar_flag = true;

    nav_msgs::msg::Odometry cnt_msg;
    cnt_msg.pose.covariance[0] = imu_cnt;
    cnt_msg.pose.covariance[1] = odom_cnt;
    cnt_msg.pose.covariance[2] = gnss_cnt;
    cnt_msg.pose.covariance[3] = lidar_cnt;
    cnt_msg.pose.covariance[4] = imu_flag;
    cnt_msg.pose.covariance[5] = odom_flag;
    cnt_msg.pose.covariance[6] = gnss_flag;
    cnt_msg.pose.covariance[7] = lidar_flag;
    pubCnt->publish(cnt_msg);

    imu_flag = false;
    odom_flag = false;
    gnss_flag = false;
    lidar_flag = false;
    
    imu_cnt = 0;
    odom_cnt = 0;
    gnss_cnt = 0;
    lidar_cnt = 0;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<sensorChecker>());

    return 0;
}
