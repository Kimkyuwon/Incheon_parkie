#include "gnssHandler.hpp"

gnssHandler::gnssHandler()
    : rclcpp::Node("gnssHandler")
    , qos_(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_services_default))
{
    Initialize();
}

gnssHandler::~gnssHandler()
{
}

int gnssHandler::Initialize()
{
    lat_std = 100;
    lon_std = 100;
    hgt_std = 100;
    N_vel_std = 100;
    E_vel_std = 100;
    U_vel_std = 100;
    roll_std = 100;
    pitch_std = 100;
    azi_std = 100;
    subGNSS = create_subscription<novatel_oem7_msgs::msg::INSPVAX>(
        "/novatel/oem7/inspvax", qos_, std::bind(&gnssHandler::callbackGnss, this, std::placeholders::_1)); 
    subINSSTDEV = create_subscription<novatel_oem7_msgs::msg::INSSTDEV>(
        "/novatel/oem7/insstdev", qos_, std::bind(&gnssHandler::callbackInsStdDev, this, std::placeholders::_1)); 
    subBestPos = create_subscription<novatel_oem7_msgs::msg::BESTPOS>(
        "/novatel/oem7/bestpos", qos_, std::bind(&gnssHandler::callbackBestPos, this, std::placeholders::_1)); 
    subResponse = create_subscription<std_msgs::msg::Bool>(
        "/response_signal_PE", qos_, std::bind(&gnssHandler::responseCallback, this, std::placeholders::_1));

    pubGnssPose = create_publisher<nav_msgs::msg::Odometry>("gnss/pose", qos_);
    pubDummyGnss = create_publisher<std_msgs::msg::Float32>("dummyGnss", qos_);
    pubReadyFlag = create_publisher<std_msgs::msg::Empty>("/ready_signal_GNSS", qos_);

    return 0;
}


void gnssHandler::callbackGnss(const novatel_oem7_msgs::msg::INSPVAX::SharedPtr gnssMsg)
{   
    std_msgs::msg::Float32 dummyMsg;
    pubDummyGnss->publish(dummyMsg);

    Eigen::Vector3d llh;
    llh(0) = (gnssMsg->latitude); llh(1) = (gnssMsg->longitude); llh(2) = gnssMsg->height;
    Eigen::Vector2d TM = LatLon2TM(llh);

    nav_msgs::msg::Odometry gnssPose;
    gnssPose.header.stamp = rclcpp::Clock().now();
    gnssPose.pose.pose.position.x = TM(0);
    gnssPose.pose.pose.position.y = TM(1);
    gnssPose.pose.pose.position.z = llh(2);
    gnssPose.twist.twist.linear.x = gnssMsg->east_velocity;
    gnssPose.twist.twist.linear.y = gnssMsg->north_velocity;
    gnssPose.twist.twist.linear.z = gnssMsg->up_velocity;
    gnssPose.pose.pose.orientation.w = deg2rad(gnssMsg->roll);  //Roll
    gnssPose.pose.pose.orientation.x = deg2rad(gnssMsg->pitch); //Pitch
    double temp_azi = -(deg2rad(gnssMsg->azimuth));
    if (temp_azi < -180)    temp_azi += 360;
    gnssPose.pose.pose.orientation.y = temp_azi; //azimuth
    gnssPose.pose.covariance[0] = gnssMsg->ins_status.status; // INS 결합 상태 --> INS_SOLUTION_GOOD = 3 이외에는 GNSS/INS 결합이 완료되지 않은 상태
    gnssPose.pose.covariance[1] = num_sv; // 가시 위성 수
    gnssPose.pose.covariance[2] = pos_type; //위치 추정 상태
    gnssPose.pose.covariance[3] = lat_std;  //Latitude STD
    gnssPose.pose.covariance[4] = lon_std;  //Longitude STD
    gnssPose.pose.covariance[5] = hgt_std;  //Height STD
    gnssPose.pose.covariance[7] = E_vel_std;    // East velocity STD
    gnssPose.pose.covariance[8] = N_vel_std;    // North velocity STD
    gnssPose.pose.covariance[9] = U_vel_std;    // Up velocity STD
    gnssPose.pose.covariance[10] = roll_std;    // roll STD
    gnssPose.pose.covariance[11] = pitch_std;   // pitch STD
    gnssPose.pose.covariance[14] = azi_std;     // azimuth STD
    pubGnssPose->publish(gnssPose);

    if (!flag_published) 
    {
        pubReadyFlag->publish(std_msgs::msg::Empty());

        RCLCPP_INFO_ONCE(this->get_logger(), "\x1b[32m" "gnssHandler Ready." "\x1b[0m");
        
        if (response_subscribed) 
        {
            flag_published = true;
        }
    }
}

void gnssHandler::callbackInsStdDev(const novatel_oem7_msgs::msg::INSSTDEV::SharedPtr stdMsg)
{   
    lat_std = stdMsg->latitude_stdev;
    lon_std = stdMsg->longitude_stdev;
    hgt_std = stdMsg->height_stdev;
    N_vel_std = stdMsg->north_velocity_stdev;
    E_vel_std = stdMsg->east_velocity_stdev;
    U_vel_std = stdMsg->up_velocity_stdev;
    roll_std = stdMsg->roll_stdev;
    pitch_std = stdMsg->pitch_stdev;
    azi_std = stdMsg->azimuth_stdev;
}

void gnssHandler::callbackBestPos(const novatel_oem7_msgs::msg::BESTPOS::SharedPtr posMsg)
{   
    num_sv = posMsg->num_svs;   // 가시 위성 개수
    /// pos type 설명
    /// INS_PSRSP = 53 --> 단일 GNSS 수신기 받은 위치 값과 INS 결합 상태 (보정 정보 사용 X)
    /// INS_PSRDIFF = 54 --> DGPS 위치 값과 INS 결합 상태
    /// INS_RTKFLOAT = 55 --> RTK Float 상태 (RTK 미지정수 추정 안됨)의 위치 값과 INS 결합 상태
    /// INS_RTKFIXED = 56 --> RTK Fixed 상태 (RTK 미지정수 추정 완료)의 위치 값과 INS 결합 상태 (가장 위치 정확도가 높음)
    /// 위치 신뢰도가 높은 상태는 INS_RTKFIXED > INS_RTKFLOAT > INS_PSRDIFF > INS_PSRSP 순
    /// 우리는 INS 결합이 완료되었을 때만 GNSS 측정치를 사용할 것이므로 다른 상태는 고려 X
    pos_type = posMsg->pos_type.type;   
}

void gnssHandler::responseCallback(const std_msgs::msg::Bool::SharedPtr rspsMsg) {

    if (rspsMsg->data) {

        response_subscribed = true;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<gnssHandler>());

    return 0;
}
