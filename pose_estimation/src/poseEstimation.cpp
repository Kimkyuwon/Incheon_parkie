#include "poseEstimation.hpp"

using namespace pose_estimation;
using namespace std;

bool gnss_enable = 0;               // 0 : GNSS disable, 1: GNSS enable 

poseEstimation::poseEstimation()
    : rclcpp::Node("poseEstimation")
    , qos_(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_services_default))

    /* variables of Kalman Filter */
    , g_state_X(6), g_res_Z(6), g_X_gnss(6), g_X_lidar(6), g_X_lidar_ori(6), g_Q_var(6), g_R_lidar(6), g_R_GNSS(6), g_TM_offset(6)
    , g_mat_Q(6,6), g_mat_H(6,6), g_mat_P(6,6), g_mat_R(6,6), g_mat_K(6,6), g_mat_I(6,6) 
{
    initialize();
}

poseEstimation::~poseEstimation() {}

int poseEstimation::initialize(){

    g_dt = 0.01;    // 100hz
    prev_time = 0;

    /* Set Kalman Filter Parameters */
    g_state_X.setZero();            // state g_state_X
    g_res_Z.setZero();              // residual g_res_Z

    g_mat_P.setZero();
    g_mat_P.diagonal() << 10000, 10000, 10000, 10000, 10000, 10000;

    g_Q_var(0) = 0.1;              // Odom variance(std) x
    g_Q_var(1) = 0.1;              // Odom variance(std) y
    g_Q_var(2) = 0.1;              // Odom variance(std) z
    g_Q_var(3) = 0.2;               // Odom variance(std) roll
    g_Q_var(4) = 0.2;               // Odom variance(std) pitch
    g_Q_var(5) = 0.2;               // Odom variance(std) yaw

    g_mat_Q.setZero();


    g_mat_H.setZero();
    g_mat_H.diagonal() << 1, 1, 1, 1, 1, 1;
    
    g_mat_R.setZero();
    g_mat_R.diagonal() << 1, 1, 1, 1, 1, 1;
    
    g_mat_I.setZero();
    g_mat_I.setIdentity();

    /* Declare Parameters */
    this->declare_parameter<double>("init_x", 0);
    this->declare_parameter<double>("init_y", 0);
    this->declare_parameter<double>("init_z", 0);
    this->declare_parameter<double>("init_roll", 0);
    this->declare_parameter<double>("init_pitch", 0);
    this->declare_parameter<double>("init_yaw", 0);
    this->get_parameter("init_x", init_x);
    this->get_parameter("init_y", init_y);
    this->get_parameter("init_z", init_z);
    this->get_parameter("init_roll", init_roll);
    this->get_parameter("init_pitch", init_pitch);
    this->get_parameter("init_yaw", init_yaw);
    
    g_TM_offset.setZero();
    g_TM_offset(0) = init_x;    g_TM_offset(1) = init_y;    
    g_state_X(0) = init_x-g_TM_offset(0); g_state_X(1) = init_y-g_TM_offset(1); g_state_X(2) = init_z;
    g_state_X(3) = deg2rad(init_roll); g_state_X(4) = deg2rad(init_pitch); g_state_X(5) = deg2rad(init_yaw);

    /* Publish & Subscribe */
    subDR = create_subscription<nav_msgs::msg::Odometry>(
        "dr/velo", qos_, std::bind(&poseEstimation::drVeloCallback, this, std::placeholders::_1));
    subGNSS = create_subscription<nav_msgs::msg::Odometry>(
        "gnss/pose", qos_, std::bind(&poseEstimation::gnssPoseCallback, this, std::placeholders::_1));
    subMeas = create_subscription<nav_msgs::msg::Path>(
        "poseMeas", qos_, std::bind(&poseEstimation::lidarPoseCallback, this, std::placeholders::_1));
    subReadyFlag = create_subscription<std_msgs::msg::Empty>(
        "/ready_signal_GNSS", qos_, std::bind(&poseEstimation::flagCallback, this, std::placeholders::_1));
    subResponse = create_subscription<std_msgs::msg::Bool>(
        "/response_signal_NL", qos_, std::bind(&poseEstimation::responseCallback, this, std::placeholders::_1));

    pubRobotPose = create_publisher<nav_msgs::msg::Odometry>("pose_COG", qos_);
    pubReadyFlag = create_publisher<std_msgs::msg::Empty>("/ready_signal_PE", qos_);
    pubResponse = create_publisher<std_msgs::msg::Bool>("/response_signal_PE", qos_);

    tf.header.frame_id = "map";
    tf.child_frame_id = "pose_COG";
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&poseEstimation::poseEstimate, this));

    return 0;
}

void poseEstimation::flagCallback(const std_msgs::msg::Empty::SharedPtr flag) {

    gnss_handler_on = true;

    auto response = std_msgs::msg::Bool();
    response.data = true;
    pubResponse->publish(response);
}

void poseEstimation::responseCallback(const std_msgs::msg::Bool::SharedPtr rspsMsg) {

    if (rspsMsg->data) 
    {
        response_subscribed = true;
    }
}

void poseEstimation::poseEstimate()
{        
    if (!flag_published ) 
    {        
        pubReadyFlag->publish(std_msgs::msg::Empty());

        RCLCPP_INFO_ONCE(this->get_logger(), "\x1b[32m" "poseEstimation Ready." "\x1b[0m");
        
        if (response_subscribed) 
        {

            flag_published = true;

            if (gnss_enable)
            {
                g_state_X(0) = g_X_gnss(0);
                g_state_X(1) = g_X_gnss(1);
            }
        }
    }

    /* Time Update of Kalman Filter */
    Eigen::VectorXd diff_update(6);
    diff_update.setZero();
    diff_update(0) = velocity(0) * g_dt;
    diff_update(1) = velocity(1) * g_dt;
    diff_update(2) = velocity(2) * g_dt;
    diff_update(3) = pi2piRad(deg2rad(rpy_inc(0) * g_dt));
    diff_update(4) = pi2piRad(deg2rad(rpy_inc(1) * g_dt));
    diff_update(5) = pi2piRad(deg2rad(rpy_inc(2) * g_dt));

    g_state_X(0) += diff_update(0);
    g_state_X(1) += diff_update(1);
    g_state_X(2) += diff_update(2);
    g_state_X(3) += diff_update(3);
    g_state_X(4) += diff_update(4);
    g_state_X(5) += diff_update(5);

    g_mat_Q.diagonal() << pow(g_Q_var(0), 2), pow(g_Q_var(1), 2), pow(g_Q_var(2), 2), 
                    pow(deg2rad(g_Q_var(3)), 2), pow(deg2rad(g_Q_var(4)), 2), pow(deg2rad(g_Q_var(5)), 2);
    g_mat_P += g_mat_Q;

    /* Broadcast Final Robot Pose */
    tf2::Quaternion quat;
    quat.setRPY(g_state_X(3), g_state_X(4), g_state_X(5));

    nav_msgs::msg::Odometry robotPose;
    robotPose.header.frame_id = "map";
    robotPose.child_frame_id = "pose_COG";
    robotPose.header.stamp = this->get_clock()->now();
    robotPose.pose.pose.position.x = g_state_X(0);
    robotPose.pose.pose.position.y = g_state_X(1);
    robotPose.pose.pose.position.z = g_state_X(2);
    robotPose.pose.pose.orientation.x = quat.x();
    robotPose.pose.pose.orientation.y = quat.y();
    robotPose.pose.pose.orientation.z = quat.z();
    robotPose.pose.pose.orientation.w = quat.w();
    robotPose.pose.covariance[0] = g_res_Z(0);                                        // X residual
    robotPose.pose.covariance[1] = g_res_Z(1);                                        // Y residual
    robotPose.pose.covariance[2] = rad2deg(g_res_Z(5));                               // Yaw residual
    robotPose.pose.covariance[3] = sqrt(g_mat_P(0, 0));                               // X covariance
    robotPose.pose.covariance[4] = sqrt(g_mat_P(1, 1));                               // Y covariance
    robotPose.pose.covariance[5] = sqrt(g_mat_P(2, 2));                               // z covariance
    robotPose.pose.covariance[7] = rad2deg(sqrt(g_mat_P(3, 3)));                      // Roll covariance
    robotPose.pose.covariance[8] = rad2deg(sqrt(g_mat_P(4, 4)));                      // Pitch covariance
    robotPose.pose.covariance[9] = rad2deg(sqrt(g_mat_P(5, 5)));                      // Yaw covariance
    robotPose.pose.covariance[10] = velocity(0);
    robotPose.pose.covariance[11] = velocity(1);
    robotPose.pose.covariance[14] = rpy_inc(2);
    robotPose.pose.covariance[15] = g_X_lidar(0);
    robotPose.pose.covariance[16] = g_X_lidar(1);
    robotPose.pose.covariance[17] = g_X_lidar(2);
    robotPose.pose.covariance[21] = g_X_lidar(3);
    robotPose.pose.covariance[22] = g_X_lidar(4);
    robotPose.pose.covariance[23] = g_X_lidar(5);

    // robotPose.twist.covariance[0] = MU_flag;                                       // Measurement update flag   
    robotPose.twist.covariance[1] = meas_info.pose.orientation.w;                     // Hessian Converged
    robotPose.twist.covariance[2] = meas_info.pose.orientation.x;                     // hdop
    robotPose.twist.covariance[3] = meas_info.pose.orientation.y;                     // pdop
    robotPose.twist.covariance[4] = meas_info.pose.orientation.z;                     // Hessian Status
    robotPose.twist.covariance[5] = meas_info.pose.position.z;                        // matching ratio
    robotPose.twist.covariance[7] = meas_info.pose.position.x;                        // localization processing time
    robotPose.twist.covariance[8] = meas_info.pose.position.y;                        // LIDAR points
    robotPose.twist.covariance[10] = hessian_cov.pose.orientation.w;                  // hessian cov x
    robotPose.twist.covariance[11] = hessian_cov.pose.orientation.x;                  // hessian cov y
    robotPose.twist.covariance[14] = hessian_cov.pose.orientation.y;                  // hessian cov z
    robotPose.twist.covariance[15] = rad2deg(hessian_cov.pose.orientation.z);         // hessian cov roll
    robotPose.twist.covariance[16] = rad2deg(hessian_cov.pose.position.x);            // hessian cov pitch
    robotPose.twist.covariance[17] = rad2deg(hessian_cov.pose.position.y);            // hessian cov yaw

    pubRobotPose->publish(robotPose);

    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(g_state_X(0), g_state_X(1), g_state_X(2)));

    tf.header.stamp = this->get_clock()->now();
    tf.transform.translation.x = g_state_X(0);
    tf.transform.translation.y = g_state_X(1);
    tf.transform.translation.z = g_state_X(2);
    tf.transform.rotation.x = quat.getX();
    tf.transform.rotation.y = quat.getY();
    tf.transform.rotation.z = quat.getZ();
    tf.transform.rotation.w = quat.getW();
    tf_broadcaster->sendTransform(tf);
    
}

void poseEstimation::initPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr init)
{
    
    // using station's marker || position from gps || .yaml parameter file
}

void poseEstimation::lidarPoseCallback(const nav_msgs::msg::Path::SharedPtr meas)
{
    double lidar_curr_time = meas->header.stamp.sec + meas->header.stamp.nanosec * 1e-9;
    double diff_lidar_time = lidar_curr_time - lidar_prev_time;
    lidar_prev_time = lidar_curr_time;

    if (diff_lidar_time < 0.09 || diff_lidar_time > 0.11)    return;

    mtx.lock();
    meas_info = meas->poses[2];
    hessian_cov = meas->poses[5];
    bool hessian_converged = meas_info.pose.orientation.w;
    bool hessian_status = meas_info.pose.orientation.z;

    double dop_scale = 0.5;
    double c = 4;
    double r = meas_info.pose.orientation.y-0.2;
    // tukey loss function
    double scale = pow(c,2)*(1-pow((1-pow((r/c),2)),3))/dop_scale; 
    if (meas_info.pose.orientation.y >= c)
    {
        scale = pow(c,2)/dop_scale;
    }
    
    if (!state_init)
    {
        if (hessian_status)
        {
            init_cnt++;
        }
        else
        {
            mtx.unlock();
            return;
        }

        if (init_cnt > 5)  // Hessian status가 10번 이상 true 시 초기화 완료 판단
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "\x1b[32m" "Position is Initialized." "\x1b[0m");
            init_cnt = 0;
            state_init = true;
        }
        else
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "\x1b[31m" "Position is not Initialized." "\x1b[0m");
        }      
    }

    g_R_lidar(0) = scale * hessian_cov.pose.orientation.w;      // LiDAR variance(std) x
    g_R_lidar(1) = scale * hessian_cov.pose.orientation.x;      // LiDAR variance(std) y
    g_R_lidar(2) = scale * hessian_cov.pose.orientation.y;      // LiDAR variance(std) z
    g_R_lidar(3) = scale * hessian_cov.pose.orientation.z;      // LiDAR variance(std) roll
    g_R_lidar(4) = scale * hessian_cov.pose.position.x;         // LiDAR variance(std) pitch
    g_R_lidar(5) = scale * hessian_cov.pose.position.y;         // LiDAR variance(std) yaw
 
    /* State at the Start of Map Matching */
    g_X_lidar_ori(0) = meas->poses[0].pose.position.x;
    g_X_lidar_ori(1) = meas->poses[0].pose.position.y;
    g_X_lidar_ori(2) = meas->poses[0].pose.position.z;
    tf2::Quaternion quat_ori;
    quat_ori.setW(meas->poses[0].pose.orientation.w);
    quat_ori.setX(meas->poses[0].pose.orientation.x);
    quat_ori.setY(meas->poses[0].pose.orientation.y);
    quat_ori.setZ(meas->poses[0].pose.orientation.z);
    tf2::Matrix3x3 mat_ori(quat_ori);
    mat_ori.getRPY(g_X_lidar_ori(3), g_X_lidar_ori(4), g_X_lidar_ori(5));

    /* Measurement of LiDAR Map Matching */
    g_X_lidar(0) = meas->poses[1].pose.position.x;
    g_X_lidar(1) = meas->poses[1].pose.position.y;
    g_X_lidar(2) = meas->poses[1].pose.position.z;
    tf2::Quaternion quat_meas;
    quat_meas.setW(meas->poses[1].pose.orientation.w);
    quat_meas.setX(meas->poses[1].pose.orientation.x);
    quat_meas.setY(meas->poses[1].pose.orientation.y);
    quat_meas.setZ(meas->poses[1].pose.orientation.z);
    tf2::Matrix3x3 mat_meas(quat_meas);
    mat_meas.getRPY(g_X_lidar(3), g_X_lidar(4), g_X_lidar(5));
    
    g_mat_R.diagonal() << pow(g_R_lidar(0), 2), pow(g_R_lidar(1), 2), pow(g_R_lidar(2), 2), 
                        pow(g_R_lidar(3), 2), pow(g_R_lidar(4), 2), pow(g_R_lidar(5), 2);
                        
    Eigen::VectorXd z(6);
    z(0) = g_X_lidar(0); z(1) = g_X_lidar(1); z(2) = g_X_lidar(2);
    z(3) = pi2piRad(g_X_lidar(3)); z(4) = pi2piRad(g_X_lidar(4)); z(5) = pi2piRad(g_X_lidar(5));

    g_mat_H.diagonal() << 1, 1, 1, 1, 1, 1;
    g_res_Z = z - g_mat_H * g_state_X;
    g_res_Z(3) = pi2piRad(g_res_Z(3)); g_res_Z(4) = pi2piRad(g_res_Z(4)); g_res_Z(5) = pi2piRad(g_res_Z(5));

    g_mat_K = g_mat_P * g_mat_H.transpose() * (g_mat_H * g_mat_P * g_mat_H.transpose() + g_mat_R).inverse();
    
    g_state_X += g_mat_K * g_res_Z; 
    g_state_X(3) = pi2piRad(g_state_X(3)); g_state_X(4) = pi2piRad(g_state_X(4)); g_state_X(5) = pi2piRad(g_state_X(5));

    g_mat_P = (g_mat_I - g_mat_K * g_mat_H) * g_mat_P;
    RCLCPP_INFO_STREAM(this->get_logger(), "\x1b[32m" "LIDAR Measurement Update." "\x1b[0m");
    mtx.unlock();
}

void poseEstimation::gnssPoseCallback(const nav_msgs::msg::Odometry::SharedPtr gnssPose)
{
    if (gnss_handler_on)
    {
        mtx.lock();
        int pos_type = gnssPose->pose.covariance[2];
        velocity(0) = gnssPose->twist.twist.linear.x;
        velocity(1) = gnssPose->twist.twist.linear.y;
        velocity(2) = gnssPose->twist.twist.linear.z;
        g_X_gnss(0) = gnssPose->pose.pose.position.x - g_TM_offset(0);
        g_X_gnss(1) = gnssPose->pose.pose.position.y - g_TM_offset(1);
        g_X_gnss(2) =  0;   // GNSS z 변수는 사용 안함
        g_X_gnss(3) = gnssPose->pose.pose.orientation.w;
        g_X_gnss(4) = gnssPose->pose.pose.orientation.x;
        g_X_gnss(5) = gnssPose->pose.pose.orientation.y;

        g_R_GNSS(0) = gnssPose->pose.covariance[3]; // GNSS variance(std) x
        g_R_GNSS(1) = gnssPose->pose.covariance[4]; // GNSS variance(std) y
        g_R_GNSS(2) = gnssPose->pose.covariance[5]; // GNSS variance(std) z
        g_R_GNSS(3) = gnssPose->pose.covariance[10]; // GNSS variance(std) roll
        g_R_GNSS(4) = gnssPose->pose.covariance[11]; // GNSS variance(std) pitch
        g_R_GNSS(5) = gnssPose->pose.covariance[14]; // GNSS variance(std) azimuth

        g_Q_var(0) = gnssPose->pose.covariance[7];
        g_Q_var(1) = gnssPose->pose.covariance[8];
        g_Q_var(2) = gnssPose->pose.covariance[9];

        //INS 결합이 안된 상태라면 GNSS 측정치 업데이트 수행 X
        if (gnssPose->pose.covariance[0] == 3)  
        {   
            gnss_enable = 1;
        }
        else
        {
            gnss_enable = 0;
        }

        if (!gnss_enable)   
        {
            mtx.unlock();
            return;
        }

        // if (g_R_GNSS(0) > 0.5 || g_R_GNSS(1) > 0.5) 
        // {
        //     return;
        // }        
        // 정지시 측정치 업데이트 수행 안함 
        if (sqrt(pow(velocity(0),2)+pow(velocity(1),2)) < 0.02 && std::fabs(rpy_inc(2)) < 0.5)
        {
            mtx.unlock();
            return;
        } 

        g_mat_R.diagonal() << pow(g_R_GNSS(0), 2), pow(g_R_GNSS(1), 2), pow(g_R_GNSS(2), 2), 
                            pow((g_R_GNSS(3)), 2), pow((g_R_GNSS(4)), 2), pow((g_R_GNSS(5)), 2);
            
        Eigen::VectorXd z(6);
        z.setZero();
        z(0) = g_X_gnss(0); z(1) = g_X_gnss(1); z(2) = g_X_gnss(2);
        z(3) = pi2piRad(g_X_gnss(3));  z(4) = pi2piRad(g_X_gnss(4));  z(5) = pi2piRad(g_X_gnss(5));  // Heading calculated by position     

        g_mat_H.diagonal() << 1, 1, 0, 1, 1, 0; //z와 heading은 사용 X (헤딩은 추후 사용 가능성 검토 후 사용)
        g_res_Z = z - g_mat_H * g_state_X;
        g_res_Z(3) = pi2piRad(g_res_Z(3));  g_res_Z(4) = pi2piRad(g_res_Z(4));  g_res_Z(5) = pi2piRad(g_res_Z(5));

        g_mat_K = g_mat_P * g_mat_H.transpose() * (g_mat_H * g_mat_P * g_mat_H.transpose() + g_mat_R).inverse();
        
        g_state_X += g_mat_K * g_res_Z; 
        g_state_X(3) = pi2piRad(g_state_X(3)); g_state_X(4) = pi2piRad(g_state_X(4)); g_state_X(5) = pi2piRad(g_state_X(5));

        g_mat_P = (g_mat_I - g_mat_K * g_mat_H) * g_mat_P;
        RCLCPP_INFO_STREAM(this->get_logger(), "\x1b[32m" "GNSS Measurement Update." "\x1b[0m");
        mtx.unlock();
    }
}

void poseEstimation::drVeloCallback(const nav_msgs::msg::Odometry::SharedPtr drVelo){
    
    double curr_time = drVelo->header.stamp.sec + drVelo->header.stamp.nanosec * 1e-9;

    if(prev_time == 0) 
    {
        prev_time = curr_time;
    }
    else 
    {
        g_dt = curr_time - prev_time;
        prev_time = curr_time;
    }    

    rpy_inc(0) = drVelo->twist.twist.angular.x;
    rpy_inc(1) = drVelo->twist.twist.angular.y;
    rpy_inc(2) = drVelo->twist.twist.angular.z;
}

int main(int argc, char **argv){

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<pose_estimation::poseEstimation>());

    return 0;
}