#include "gnssHandler.hpp"

bool NMEA_Enable = 1;   // 0 : disable = x, y from UBX msg, 1: enable 
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
    subGNSS = create_subscription<novatel_oem7_msgs::msg::INSPVA>(
        "/novatel/oem7/inspva", qos_, std::bind(&gnssHandler::callbackGnss, this, std::placeholders::_1)); // ublox_gps_node/navpvt
    subNMEA = create_subscription<sensor_msgs::msg::NavSatFix>(
        "fix/mrp", qos_, std::bind(&gnssHandler::callbackNMEA, this, std::placeholders::_1));
    subResponse = create_subscription<std_msgs::msg::Bool>(
        "/response_signal_PE", qos_, std::bind(&gnssHandler::responseCallback, this, std::placeholders::_1));

    pubGnssPose = create_publisher<nav_msgs::msg::Odometry>("gnss/pose", qos_);
    pubDummyGnss = create_publisher<std_msgs::msg::Float32>("dummyGnss", qos_);
    pubReadyFlag = create_publisher<std_msgs::msg::Empty>("/ready_signal_GNSS", qos_);

    return 0;
}

// /*
void gnssHandler::callbackGnss(const novatel_oem7_msgs::msg::INSPVA::SharedPtr gnssMsg)
{   
    std_msgs::msg::Float32 dummyMsg;
    pubDummyGnss->publish(dummyMsg);

    // double latitude, longitude, height, x, y;
    // latitude = (gnssMsg->lat) * 1e-7; longitude = (gnssMsg->lon) * 1e-7; height = gnssMsg->height;
    // LatLon2TM(latitude, longitude, x, y);

    Eigen::Vector3d llh;
    llh(0) = (gnssMsg->latitude) * 1e-7; llh(1) = (gnssMsg->longitude) * 1e-7; llh(2) = gnssMsg->height;
    Eigen::Vector2d TM = LatLon2TM(llh);

    nav_msgs::msg::Odometry gnssPose;
    gnssPose.header.stamp = rclcpp::Clock().now();
    // gnssPose.pose.pose.position.x = x;
    // gnssPose.pose.pose.position.y = y;
    // gnssPose.pose.pose.position.z = height;
    gnssPose.pose.pose.position.x = TM(0);
    gnssPose.pose.pose.position.y = TM(1);
    gnssPose.pose.pose.position.z = llh(2);
    gnssPose.pose.covariance[0] = 0; //gnssMsg->ins_status; //flags;       // CARRIER_PHASE_FLOAT = 64, CARRIER_PHASE_FIXED = 128
    gnssPose.pose.covariance[1] = 0; // //gnssMsg->num_sv;
    gnssPose.pose.covariance[2] = 0; //gnssMsg->p_dop * 1e-2;
    gnssPose.pose.covariance[3] = 0; //gnssMsg->h_acc * 1e-3;
    gnssPose.pose.covariance[4] = 0; //gnssMsg->v_acc * 1e-3;
    gnssPose.pose.covariance[5] = 0; //gnssMsg->head_acc * 1e-5;

    NMEA_Enable = 0;
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
// */
void gnssHandler::responseCallback(const std_msgs::msg::Bool::SharedPtr rspsMsg) {

    if (rspsMsg->data) {

        response_subscribed = true;
    }
}

void gnssHandler::callbackNMEA(const sensor_msgs::msg::NavSatFix::SharedPtr nmeaMsg)
{
    std_msgs::msg::Float32 dummyMsg;
    pubDummyGnss->publish(dummyMsg);

    if(NMEA_Enable) {
        
        // double latitude, longitude, height, x, y;
        // latitude = (nmeaMsg->latitude) * 1e-7; longitude = (nmeaMsg->longitude) * 1e-7; height = nmeaMsg->altitude;
        // LatLon2TM(latitude, longitude, x, y);
        
        Eigen::Vector3d llh;
        llh(0) = (nmeaMsg->latitude); llh(1) = (nmeaMsg->longitude); llh(2) = nmeaMsg->altitude;
        Eigen::Vector2d TM = LatLon2TM(llh);

        nav_msgs::msg::Odometry gnssPose;
        gnssPose.header.stamp = rclcpp::Clock().now();
        // gnssPose.pose.pose.position.x = x;
        // gnssPose.pose.pose.position.y = y;
        // gnssPose.pose.pose.position.z = height;
        gnssPose.pose.pose.position.x = TM(0);
        gnssPose.pose.pose.position.y = TM(1);
        gnssPose.pose.pose.position.z = llh(2);
        gnssPose.pose.covariance[0] = sqrt(nmeaMsg->position_covariance[0]);   
        gnssPose.pose.covariance[1] = sqrt(nmeaMsg->position_covariance[4]);    
        gnssPose.pose.covariance[2] = sqrt(nmeaMsg->position_covariance[8]);  
        // gnssPose.pose.covariance[1] = gnssMsg->num_sv;
        // gnssPose.pose.covariance[2] = gnssMsg->p_dop;
        // gnssPose.pose.covariance[3] = gnssMsg->h_acc;
        // gnssPose.pose.covariance[4] = gnssMsg->v_acc;
        // gnssPose.pose.covariance[5] = gnssMsg->head_acc;
        
        pubGnssPose->publish(gnssPose);
    }

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

/*void gnssHandler::LatLon2TM(double lat, double lon, double &x, double &y) {
    // Set Ellips factor
    lat = deg2rad(lat);
    lon = deg2rad(lon);

    double m_arMajor = 6378137.0;
    double m_arMinor = 6356752.3142;

    // Set System Factor
    double m_arScaleFactor = 1;

    // 중부 원점
    double m_arLonCenter = deg2rad(127.0);
    double m_arLatCenter = deg2rad(38.0);
    double m_arFalseNorthing = 600000; // 1524.0); to X axis (+)
    double m_arFalseEasting = 200000; // -1760.0); to Y axis (-)

    // Set Internal Values
    double temp = m_arMinor / m_arMajor;

    double m_dDstEs = 1.0 - temp * temp;
    double m_dDstE = sqrt(m_dDstEs);
    double m_dDstE0 = 1.0 - 0.25 * m_dDstEs * (1.0 + m_dDstEs / 16.0 * (3.0 + 1.25 * m_dDstEs));
    double m_dDstE1 = 0.375 * m_dDstEs * (1.0 + 0.25 * m_dDstEs * (1.0 + 0.46875 * m_dDstEs));
    double m_dDstE2 = 0.05859375 * m_dDstEs * m_dDstEs * (1.0 + 0.75 * m_dDstEs);
    double m_dDstE3 = m_dDstEs * m_dDstEs * m_dDstEs * (35.0 / 3072.0);
    double m_dDstMl0 = m_arMajor * m_dDstE0 * m_arLatCenter - m_dDstE1 * sin(2.0 * m_arLatCenter) + m_dDstE2 * sin(4.0 * m_arLatCenter) - m_dDstE3 * sin(6.0 * m_arLatCenter);
    double m_dDstEsp = m_dDstEs / (1.0 - m_dDstEs);

    double m_dDstInd;
    double delta_lon;           // Delta longitude (Given longitude - center longitude)
    double sin_phi, cos_phi;    // sin and cos value
    double al, als;             // temporary values
    double b, c, t, tq;         // temporary values
    double con, n, ml;          // cone constant, small m

    if (m_dDstEs < 0.00001)
        m_dDstInd = 1.0;
    else {
        m_dDstInd = 0.0;

        // LL to TM Forward equations from here
        delta_lon = lon - m_arLonCenter;
        sin_phi = sin(lat);
        cos_phi = cos(lat);

        if (m_dDstInd != 0)
            b = cos_phi * sin(delta_lon);
            if ((fabs(fabs(b) - 1.0)) < 0.0000000001) {;}
        else {
            b = 0;
            x = 0.5 * m_arMajor * m_arScaleFactor * log((1.0 + b) / (1.0 - b));
            con = acos(cos_phi * cos(delta_lon) / sqrt(1.0 - b * b));
            if (lat < 0) {
                con = -con;
                y = m_arMajor * m_arScaleFactor * (con - m_arLatCenter);
            }
        }
    }

    al = cos_phi * delta_lon;
    als = al * al;
    c = m_dDstEsp * cos_phi * cos_phi;
    tq = tan(lat);
    t = tq * tq;
    con = 1.0 - m_dDstEs * sin_phi * sin_phi;
    n = m_arMajor / sqrt(con);
    ml = m_arMajor * m_dDstE0 * lat - m_dDstE1 * sin(2.0 * lat) + m_dDstE2 * sin(4.0 * lat) - m_dDstE3 * sin(6.0 * lat);
    y = m_arScaleFactor * n * al * (1.0 + als / 6.0 * (1.0 - t + c + als / 20.0 * (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * m_dDstEsp))) + m_arFalseEasting;
    x = m_arScaleFactor * (ml - m_dDstMl0 + n * tq * (als * (0.5 + als / 24.0 * (5.0 - t + 9.0 * c + 4.0 * c * c + als / 30.0 * (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * m_dDstEsp))))) + m_arFalseNorthing;

}*/

/*
void gnssHandler::LatLon2TM(double lat, double lon, double &x, double &y) {

    // check input lat, lon (deg or rad)
    lat = deg2rad(lat);

    double a = 6378137;
    double b = 6356752.314250;
    double f = 0.00335281;
    double k0 = 1;
    double esqure = (pow(a, 2) - pow(b, 2)) / pow(a, 2);
    double epsqure = (pow(a, 2) - pow(b, 2)) / pow(b, 2);
    double dY = 200000;
    double dX = 600000;
    double T = pow(tan(lat), 2);
    double C = esqure * pow(cos(lat), 2) / (1 - esqure);
    double ramda = deg2rad(lon);
    double ramda0 = deg2rad(127);  //중부지점 투영원점 경도
    double A = (ramda - ramda0) * cos(lat);
    double N = a / sqrt(1 - (esqure * pow(sin(lat), 2)));

    double Msub0 = (1 - (esqure / 4) - (3 * pow(esqure, 2) / 64) - (5 * pow(esqure, 3) / 256)) * lat;
    double Msub1 = ((3 * esqure / 8) + (3 * pow(esqure, 2) / 32) + (45 * pow(esqure, 3) / 1024)) * sin(2 * lat);
    double Msub2 = ((15 * pow(esqure, 2) / 256) + (45 * pow(esqure, 3) / 1024)) * sin(4 * lat);
    double Msub3 = 35 * pow(esqure, 3) / 3072 * sin(6 * lat);
    double M = a * (Msub0 - Msub1 + Msub2 - Msub3);

    double Ysub0 = pow(A, 3) * (1 - T + C) / 6;
    double Ysub1 = pow(A, 5) * (5 - (18 * T) + pow(T, 2) + (72 * C) - (58 * epsqure)) / 120;
    double Ysub2 = A + Ysub0 + Ysub1;
    y = dY + (k0 * N * Ysub2);

    double Xsub0 = (pow(A, 2) / 2) + (pow(A, 4) * (5 - T + (9 * C) + (4 * pow(C, 2))) / 24);
    double Xsub1 = pow(A, 6) * (61 - (58 * T) + pow(T, 2) + (600 * C) - (330 * epsqure)) / 720;
    double M0 = 4207498.019266;
    double Xsub2 = N * tan(lat) * (Xsub0 + Xsub1);
    x = dX + (k0 * (M - M0 + Xsub2));
}*/

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<gnssHandler>());

    return 0;
}
