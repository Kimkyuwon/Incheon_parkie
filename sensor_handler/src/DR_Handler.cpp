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

    subIMU = create_subscription<sensor_msgs::msg::Imu>(
        "imu", qos_, std::bind(&DR_Handler::imuHandler, this, std::placeholders::_1));
    subOdom = create_subscription<nav_msgs::msg::Odometry>(
        "/novatel/oem7/odom", qos_, std::bind(&DR_Handler::odomHandler, this, std::placeholders::_1)); // odom

    pubDR = create_publisher<nav_msgs::msg::Odometry>("dr/velo", qos_);
    pubDummyImu = create_publisher<std_msgs::msg::Float32>("dummyImu", qos_);
    pubDummyOdom = create_publisher<std_msgs::msg::Float32>("dummyOdom", qos_);

    return 0;
}

void DR_Handler::imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg){

    std_msgs::msg::Float32 dummyMsg;
    pubDummyImu->publish(dummyMsg);

    double curr_time_imu = imuMsg->header.stamp.sec + imuMsg->header.stamp.nanosec * 1e-9;
    
    if (prev_time_imu == 0) {
        prev_time_imu =  curr_time_imu;
    }
    else {
        dt_imu = curr_time_imu - prev_time_imu;
        prev_time_imu = curr_time_imu;
    }
    
    // add stop conditions
    if (bias_cnt < 100){
        bias_cnt++;
        roll_bias = ((roll_bias * (bias_cnt - 1)) + imuMsg->angular_velocity.x) / bias_cnt;
        pitch_bias = ((pitch_bias * (bias_cnt - 1)) + imuMsg->angular_velocity.y) / bias_cnt;
        yaw_bias = ((yaw_bias * (bias_cnt - 1)) + imuMsg->angular_velocity.z) / bias_cnt;
    }

    gyro(0) = imuMsg->angular_velocity.x - roll_bias;
    gyro(1) = imuMsg->angular_velocity.y - pitch_bias;
    gyro(2) = imuMsg->angular_velocity.z - yaw_bias;
    acc(0) = imuMsg->linear_acceleration.x;
    acc(1) = imuMsg->linear_acceleration.y;
    acc(2) = imuMsg->linear_acceleration.z;

    // need to be verified
    accel_x = (acc(0) + 9.81 * sin(gyro(1))) * cos(gyro(1));
    accel_y = (acc(0) + 9.81 * sin(gyro(0))) * cos(gyro(0));
    velo_x = velo_x_temp + accel_x * dt_imu;
    velo_y = velo_y_temp + accel_y * dt_imu;

    /* get DR data from IMU, only if Odometry data not published (temporary) */
    if (!odom_received)
    {
        
        double roll_inc, pitch_inc, yaw_inc;
        roll_inc = rad2deg(gyro(0)); pitch_inc = rad2deg(gyro(1)); yaw_inc = rad2deg(gyro(2));

        nav_msgs::msg::Odometry imuVelo;
        imuVelo.header.stamp = imuMsg->header.stamp;
        imuVelo.twist.twist.linear.x = velo_x;
        imuVelo.twist.twist.linear.y = velo_y;
        imuVelo.twist.twist.angular.x = roll_inc;
        imuVelo.twist.twist.angular.y = pitch_inc;
        imuVelo.twist.twist.angular.z = yaw_inc;

        pubDR->publish(imuVelo);
    }
}

void DR_Handler::odomHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg){

    std_msgs::msg::Float32 dummyMsg;
    pubDummyOdom->publish(dummyMsg);
    
    odom_received = true;
    
    /* linear velocity from Odom, angular velocity from IMU, linear acceleration from IMU(just store at covariance) */
    nav_msgs::msg::Odometry drVelo;
    drVelo.header.stamp = odomMsg->header.stamp;
    drVelo.twist.twist.linear.x = odomMsg->twist.twist.linear.y; //odomMsg->twist.twist.linear.x;
    drVelo.twist.twist.linear.y = odomMsg->twist.twist.linear.x; //odomMsg->twist.twist.linear.y;
    drVelo.twist.twist.angular.x = 0.0; //rad2deg(gyro(0));
    drVelo.twist.twist.angular.y = 0.0; //rad2deg(gyro(1));
    drVelo.twist.twist.angular.z = rad2deg(odomMsg->twist.twist.angular.z); //  //-rad2deg(gyro(2)); // odomMsg->twist.twist.angular.z

    pubDR->publish(drVelo);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<DR_Handler>());

    return 0;
}
