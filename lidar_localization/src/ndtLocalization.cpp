#include "ndtLocalization.hpp"
/*
1. pcl include : pcl-1.10/pcl/~
2. ros : rclcpp
3. ~_msgs/msg/[A-Z]~~ : ~_msgs/msg/msg/[a-z]~~
4. opencv : not used
5. tf : tf2
*/
using std::atan2;
using std::cos;
using std::sin;
using namespace std;
using namespace robotLocalization;

static const rmw_qos_profile_t rmw_qos_profile_for_lidar =
{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

ndtLocalization::ndtLocalization()
: rclcpp::Node("ndtLocalization")
, qos_(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_services_default))
, qos_q1(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_for_lidar))
, laserCloudMap (new pcl::PointCloud<PointType> ())
, laserCloudVisMap (new pcl::PointCloud<PointType> ())
, laserCloud (new pcl::PointCloud<PointType> ())
, laserCloudGlobal (new pcl::PointCloud<PointType> ())
, state_meas(6)
, hessian_diag(6)
, g_TM_offset(6)
, kdtree(new pcl::KdTreeFLANN<PointType>())
{
    Initialize();
}
ndtLocalization::~ndtLocalization(){}
int ndtLocalization::Initialize(){
    this->declare_parameter<double>("voxel_size", 0);
    this->declare_parameter<double>("map_voxel_size", 0);
    this->declare_parameter<std::string>("map_directory", "/");
    this->declare_parameter<std::string>("matching_mode", "GICP");
    this->declare_parameter<double>("init_x", 0);
    this->declare_parameter<double>("init_y", 0);
    this->declare_parameter<double>("init_z", 0);
    this->declare_parameter<double>("init_roll", 0);
    this->declare_parameter<double>("init_pitch", 0);
    this->declare_parameter<double>("init_yaw", 0);
    this->declare_parameter<double>("l2b_roll", 0);
    this->declare_parameter<double>("l2b_pitch", 0);
    this->declare_parameter<double>("l2b_yaw", 0);
    this->declare_parameter<double>("l2b_x", 0);
    this->declare_parameter<double>("l2b_y", 0);
    this->declare_parameter<double>("l2b_z", 0);
    this->declare_parameter<float>("map_matching_range", 0);
    this->declare_parameter<int>("iter_num", 0);
    this->declare_parameter<int>("thread_num", 0);
    this->declare_parameter<double>("step_size", 0);
    this->declare_parameter<double>("epsilon", 0);
    this->declare_parameter<double>("resolution", 0);
    this->declare_parameter<bool>("debug_flag", false);

    this->get_parameter("voxel_size", voxel_size);
    this->get_parameter("map_voxel_size", map_voxel_size);
    this->get_parameter("map_directory", map_directory);
    this->get_parameter("matching_mode", matching_mode);
    this->get_parameter("init_x", init_x);
    this->get_parameter("init_y", init_y);
    this->get_parameter("init_z", init_z);
    this->get_parameter("init_roll", init_roll);
    this->get_parameter("init_pitch", init_pitch);
    this->get_parameter("init_yaw", init_yaw);
    this->get_parameter("l2b_roll", l2b_roll);
    this->get_parameter("l2b_pitch", l2b_pitch);
    this->get_parameter("l2b_yaw", l2b_yaw);
    this->get_parameter("l2b_x", l2b_x);
    this->get_parameter("l2b_y", l2b_y);
    this->get_parameter("l2b_z", l2b_z);
    this->get_parameter("map_matching_range", mapMatchingRange);
    this->get_parameter("iter_num", iter_num);
    this->get_parameter("thread_num", thread_num);
    this->get_parameter("step_size", step_size);
    this->get_parameter("epsilon", epsilon);
    this->get_parameter("resolution", resolution);
    this->get_parameter("debug_flag", debug_flag);

    g_TM_offset.setZero();
    g_TM_offset(0) = init_x;    g_TM_offset(1) = init_y;    g_TM_offset(2) = init_z;
    g_TM_offset(5) = init_yaw;
    Eigen::Matrix4d TM_TF(Eigen::Matrix4d::Identity());
    TM_TF(0,3) = 207590-g_TM_offset(0);
    TM_TF(1,3) = 533980-g_TM_offset(1);
    if (pcl::io::loadPCDFile(map_directory, *laserCloudMap) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Map file cannot open.\n");
        return 0;
    }
    pcl::transformPointCloud(*laserCloudMap, *laserCloudMap, TM_TF);

    Eigen::Matrix3f R;
    L2B_TF.setIdentity();
    R = Eigen::AngleAxisf(deg2rad(l2b_roll), Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(deg2rad(l2b_pitch), Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(deg2rad(l2b_yaw), Eigen::Vector3f::UnitZ());
    L2B_TF.block(0,0,3,3) = R;
    L2B_TF(0,3) = l2b_x;  L2B_TF(1,3) = l2b_y;  L2B_TF(2,3) = l2b_z;

    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setInputCloud(laserCloudMap);
    downSizeFilter.setLeafSize(map_voxel_size, map_voxel_size, map_voxel_size);
    downSizeFilter.filter(*laserCloudMap);
    pcl::removeNaNFromPointCloud(*laserCloudMap, *laserCloudMap, indiceLet);
    indiceLet.clear();

    pcl::VoxelGrid<PointType> downSizeFilter_vis;
    downSizeFilter_vis.setInputCloud(laserCloudMap);
    downSizeFilter_vis.setLeafSize(0.5, 0.5, 0.5);
    downSizeFilter_vis.filter(*laserCloudVisMap);
    pcl::removeNaNFromPointCloud(*laserCloudVisMap, *laserCloudVisMap, indiceLet);
    indiceLet.clear();

    kdtree->setInputCloud(laserCloudVisMap);

    // need to remove (Goalie)
    pcl::ConditionAnd<PointType>::Ptr range_cond (new pcl::ConditionAnd<PointType>());
    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("z", pcl::ComparisonOps::LT, 2.5)));

    // build the filter
    pcl::ConditionalRemoval<PointType> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(laserCloudMap);
    condrem.setKeepOrganized (true);
    // apply filter
    condrem.filter (*laserCloudMap); 

    if (matching_mode == "NDT")
    {
        ndt.setInputTarget(laserCloudMap);
        ndt.setMaximumIterations(iter_num);
        ndt.setTransformationEpsilon(epsilon);
        ndt.setResolution(resolution);
        ndt.setStepSize(step_size);
    }
    else if (matching_mode == "NDT_OMP")
    {
        ndtomp.setInputTarget(laserCloudMap);
        ndtomp.setMaximumIterations(iter_num);
        ndtomp.setTransformationEpsilon(epsilon);
        ndtomp.setResolution(resolution);
        ndtomp.setStepSize(step_size);
        ndtomp.setNumThreads(thread_num);
        ndtomp.setNeighborhoodSearchMethod(pclompm::DIRECT7);
    }


    initPos.setZero();
    initAtt.setZero();
    Eigen::Vector2d l2b_init_vec;
    l2b_init_vec(0) = l2b_x*cos(deg2rad(init_yaw)) - l2b_y*sin(deg2rad(init_yaw));
    l2b_init_vec(1) = l2b_x*sin(deg2rad(init_yaw)) + l2b_y*cos(deg2rad(init_yaw));
    initPos(0) = (init_x-g_TM_offset(0)) + l2b_init_vec(0);   initPos(1) = (init_y-g_TM_offset(1)) + l2b_init_vec(1);  initPos(2) = init_z-g_TM_offset(2);
    initAtt(0) = init_roll;   initAtt(1) = init_pitch;   initAtt(2) = init_yaw;
    
    state_TF(0,3) = initPos(0);
    state_TF(1,3) = initPos(1);
    state_TF(2,3) = initPos(2);
    Eigen::AngleAxisf rollAngle(deg2rad(initAtt(2)), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(deg2rad(initAtt(1)), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(deg2rad(initAtt(0)), Eigen::Vector3f::UnitX());

    Eigen::Quaternionf q =  yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3f R_init = q.matrix();
    state_TF.block(0,0,3,3) = R_init;

    subState = create_subscription<nav_msgs::msg::Odometry>(
        "pose_COG", qos_, std::bind(&ndtLocalization::currStateHandler, this, std::placeholders::_1));
    subLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar_points", qos_q1, std::bind(&ndtLocalization::laserCloudHandler, this, std::placeholders::_1));
    subReadyFlagLH = create_subscription<std_msgs::msg::Empty>(
        "/ready_signal_LH", qos_, std::bind(&ndtLocalization::flagCallbackLH, this, std::placeholders::_1));

    subReadyFlagPE = create_subscription<std_msgs::msg::Empty>(
        "/ready_signal_PE", qos_, std::bind(&ndtLocalization::flagCallbackPE, this, std::placeholders::_1));

    pubLaserCloudMap = create_publisher<sensor_msgs::msg::PointCloud2>("previousMap", qos_);
    pubLaserCloud = create_publisher<sensor_msgs::msg::PointCloud2>("matchingPoints", qos_);
    pubMeasurementInfo = create_publisher<nav_msgs::msg::Path>("poseMeas", qos_);
    pubResponse = create_publisher<std_msgs::msg::Bool>("/response_signal_NL", qos_);

    return 0;
}

void ndtLocalization::pubSurroundMap()
{
    pcl::ConditionAnd<PointType>::Ptr range_cond (new pcl::ConditionAnd<PointType> ());
    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("x", pcl::ComparisonOps::GT, state_TF(0,3)-mapMatchingRange)));
    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("x", pcl::ComparisonOps::LT, state_TF(0,3)+mapMatchingRange)));
    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("y", pcl::ComparisonOps::GT, state_TF(1,3)-mapMatchingRange)));
    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("y", pcl::ComparisonOps::LT, state_TF(1,3)+mapMatchingRange)));

    // build the filter
    pcl::ConditionalRemoval<PointType> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(laserCloudVisMap);
    condrem.setKeepOrganized (true);

    pcl::PointCloud<PointType>::Ptr laserCloudLocalMap (new pcl::PointCloud<PointType> ());
    // apply filter
    condrem.filter (*laserCloudLocalMap);
    pcl::removeNaNFromPointCloud(*laserCloudLocalMap, *laserCloudLocalMap, indiceLet);
    sensor_msgs::msg::PointCloud2 laserCloudMapMsg;
    pcl::toROSMsg(*laserCloudLocalMap, laserCloudMapMsg);
    laserCloudMapMsg.header.stamp = rclcpp::Time(timeLaserCloud,timeLaserCloudNano);
    laserCloudMapMsg.header.frame_id = "map";
    pubLaserCloudMap->publish(laserCloudMapMsg);
}

void ndtLocalization::pubMeasurement()
{
    Eigen::Matrix4f TF (Eigen::Matrix4f::Identity());
    Eigen::Matrix3f R;
    R =  Eigen::AngleAxisf(state_meas(5), Eigen::Vector3f::UnitZ())
        * Eigen::AngleAxisf(state_meas(4), Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(state_meas(3), Eigen::Vector3f::UnitX());

    TF.block(0,0,3,3) = R;
    TF(0,3) = state_meas(0); TF(1,3) = state_meas(1);  TF(2,3) = state_meas(2);

    Eigen::Vector3f L2B_vec;
    L2B_vec(0) = L2B_TF(0,3);
    L2B_vec(1) = L2B_TF(1,3);
    L2B_vec(2) = L2B_TF(2,3);
    L2B_vec = R * L2B_vec;
    TF(0,3) = TF(0,3) + L2B_vec(0);
    TF(1,3) = TF(1,3) + L2B_vec(1);
    TF(2,3) = TF(2,3) + L2B_vec(2);

    Eigen::VectorXd meas(6);
    meas(0) = TF(0,3);
    meas(1) = TF(1,3);
    meas(2) = TF(2,3);
    Eigen::Matrix3f measOrigin_R = TF.block(0,0,3,3);

    Eigen::Quaternionf q(measOrigin_R);
    tf2::Quaternion q_measOrigin;
    q_measOrigin.setW(q.w());
    q_measOrigin.setX(q.x());
    q_measOrigin.setY(q.y());
    q_measOrigin.setZ(q.z());
    tf2::Matrix3x3 m2(q_measOrigin);
    m2.getRPY(meas(3), meas(4), meas(5));


    double matching_ratio = 0;

    matching_ratio = static_cast<float>(matching_points_num) / 3000;    //max_points_num;  //laserCloudGlobal->points.size();  //4000;

    measPath.poses.clear();
    geometry_msgs::msg::PoseStamped odomOrigin;
    tf2::Quaternion q_origin;
    q_origin.setRPY(meas(3), meas(4), meas(5));
    odomOrigin.header.frame_id = "map";
    odomOrigin.header.stamp = rclcpp::Time(timeLaserCloud,timeLaserCloudNano);
    odomOrigin.pose.orientation.x = q_origin.x();
    odomOrigin.pose.orientation.y = q_origin.y();
    odomOrigin.pose.orientation.z = q_origin.z();
    odomOrigin.pose.orientation.w = q_origin.w();
    odomOrigin.pose.position.x = meas(0);
    odomOrigin.pose.position.y = meas(1);
    odomOrigin.pose.position.z = meas(2);
    measPath.header.stamp = rclcpp::Time(timeLaserCloud,timeLaserCloudNano);
    measPath.header.frame_id = "map";
    measPath.poses.push_back(odomOrigin);

    Eigen::Matrix3f meas_R;
    meas_R = meas_TF.block(0,0,3,3);
    Eigen::Vector3f L2B_vec2;
    L2B_vec2(0) = L2B_TF(0,3);
    L2B_vec2(1) = L2B_TF(1,3);
    L2B_vec2(2) = L2B_TF(2,3);
    L2B_vec2 = meas_R * L2B_vec2;
    Eigen::Vector3d meas_TF_cog;
    meas_TF_cog(0) = meas_TF(0,3) + L2B_vec2(0);
    meas_TF_cog(1) = meas_TF(1,3) + L2B_vec2(1);
    meas_TF_cog(2) = meas_TF(2,3) + L2B_vec2(2);

    geometry_msgs::msg::PoseStamped odomMeas;
    Eigen::Quaternionf q_meas(meas_R);
    odomMeas.header.frame_id = "map";
    odomMeas.header.stamp = rclcpp::Time(timeLaserCloud,timeLaserCloudNano);
    odomMeas.pose.orientation.x = q_meas.x();
    odomMeas.pose.orientation.y = q_meas.y();
    odomMeas.pose.orientation.z = q_meas.z();
    odomMeas.pose.orientation.w = q_meas.w();
    odomMeas.pose.position.x = meas_TF_cog(0);
    odomMeas.pose.position.y = meas_TF_cog(1);
    odomMeas.pose.position.z = meas_TF_cog(2);
    measPath.header.stamp = rclcpp::Time(timeLaserCloud,timeLaserCloudNano);
    measPath.header.frame_id = "map";
    measPath.poses.push_back(odomMeas);

    geometry_msgs::msg::PoseStamped odomInfo;
    odomInfo.header.stamp = rclcpp::Time(timeLaserCloud, timeLaserCloudNano);
    odomInfo.pose.orientation.x = hdop;
    odomInfo.pose.orientation.y = pdop;
    odomInfo.pose.orientation.z = hessian_Status;
    odomInfo.pose.orientation.w = hessian_Converged;

    odomInfo.pose.position.x = wholeLocalizationProcessTime;
    odomInfo.pose.position.y = laserCloudGlobal->points.size();
    odomInfo.pose.position.z = matching_ratio;
    measPath.header.stamp = rclcpp::Time(timeLaserCloud, timeLaserCloudNano);
    measPath.header.frame_id = "map";
    measPath.poses.push_back(odomInfo); 

    geometry_msgs::msg::PoseStamped odomMeas2;
    odomMeas2.header.frame_id = "map";
    odomMeas2.header.stamp = rclcpp::Time(timeLaserCloud,timeLaserCloudNano);
    odomMeas2.pose.orientation.x = q_meas.x();
    odomMeas2.pose.orientation.y = q_meas.y();
    odomMeas2.pose.orientation.z = q_meas.z();
    odomMeas2.pose.orientation.w = q_meas.w();
    odomMeas2.pose.position.x = meas_TF(0,3);
    odomMeas2.pose.position.y = meas_TF(1,3);
    odomMeas2.pose.position.z = meas_TF(2,3);
    measPath.header.stamp = rclcpp::Time(timeLaserCloud,timeLaserCloudNano);
    measPath.header.frame_id = "map";
    measPath.poses.push_back(odomMeas2);

    Eigen::Matrix3f state_R;
    state_R = state_TF.block(0,0,3,3);
    Eigen::Quaternionf q_state(state_R);

    geometry_msgs::msg::PoseStamped odomState;
    odomState.header.frame_id = "map";
    odomState.header.stamp = rclcpp::Time(timeLaserCloud,timeLaserCloudNano);
    odomState.pose.orientation.x = q_state.x();
    odomState.pose.orientation.y = q_state.y();
    odomState.pose.orientation.z = q_state.z();
    odomState.pose.orientation.w = q_state.w();
    odomState.pose.position.x = state_TF(0,3);
    odomState.pose.position.y = state_TF(1,3);
    odomState.pose.position.z = state_TF(2,3);
    measPath.header.stamp = rclcpp::Time(timeLaserCloud,timeLaserCloudNano);
    measPath.header.frame_id = "map";
    measPath.poses.push_back(odomState);

    geometry_msgs::msg::PoseStamped Hessian_cov;
    Hessian_cov.header.frame_id = "map";
    Hessian_cov.header.stamp = rclcpp::Time(timeLaserCloud, timeLaserCloudNano);
    Hessian_cov.pose.orientation.w = sqrt(fabs(hessian_diag(0))) * 10;
    Hessian_cov.pose.orientation.x = sqrt(fabs(hessian_diag(1))) * 10;
    Hessian_cov.pose.orientation.y = sqrt(fabs(hessian_diag(2))) * 10;
    Hessian_cov.pose.orientation.z = sqrt(fabs(hessian_diag(3))) * 10;
    Hessian_cov.pose.position.x = sqrt(fabs(hessian_diag(4))) * 10;
    Hessian_cov.pose.position.y = sqrt(fabs(hessian_diag(5))) * 10;
    measPath.header.stamp = rclcpp::Time(timeLaserCloud, timeLaserCloudNano);
    measPath.header.frame_id = "map";
    measPath.poses.push_back(Hessian_cov);

    pubMeasurementInfo->publish(measPath);
}

void ndtLocalization::flagCallbackLH(const std_msgs::msg::Empty::SharedPtr flag) {

    lidar_handler_on = true;
    // std::cout << "\033[1;32m" << " Front NDT Localization Subscribed LiDAR Handler Flag " << "\033[0m" << std::endl;
}

void ndtLocalization::flagCallbackPE(const std_msgs::msg::Empty::SharedPtr flag) {

    pose_estimation_on = true;

    if (lidar_handler_on && pose_estimation_on) {

        auto response = std_msgs::msg::Bool();
        response.data = true;
        pubResponse->publish(response);

        RCLCPP_INFO_ONCE(this->get_logger(), "\x1b[32m" "ndtLocalization Ready." "\x1b[0m");
    }
}

void ndtLocalization::currStateHandler(const nav_msgs::msg::Odometry::SharedPtr state)
{
    state_TF.setIdentity();
    state_TF(0,3) = state->pose.pose.position.x;
    state_TF(1,3) = state->pose.pose.position.y;
    state_TF(2,3) = state->pose.pose.position.z;
    Eigen::Quaternionf q_state;
    q_state.w() = state->pose.pose.orientation.w;
    q_state.x() = state->pose.pose.orientation.x;
    q_state.y() = state->pose.pose.orientation.y;
    q_state.z() = state->pose.pose.orientation.z;
    Eigen::Matrix3f state_R = q_state.toRotationMatrix();
    state_TF.block(0,0,3,3) = state_R;
    Eigen::Vector3f L2B_vec;
    L2B_vec(0) = -L2B_TF(0,3);
    L2B_vec(1) = -L2B_TF(1,3);
    L2B_vec(2) = -L2B_TF(2,3);
    L2B_vec = state_R * L2B_vec;
    state_TF(0,3) = state_TF(0,3) + L2B_vec(0);
    state_TF(1,3) = state_TF(1,3) + L2B_vec(1);
    state_TF(2,3) = state_TF(2,3) + L2B_vec(2);
}

void ndtLocalization::laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
{
    timeLaserCloud = laserCloudMsg->header.stamp.sec;
    timeLaserCloudNano = laserCloudMsg->header.stamp.nanosec;
    pcl::PointCloud<PointType>::Ptr laserCloudVisual(new pcl::PointCloud<PointType>());

    pcl::PointCloud<PointType>::Ptr laserCloudTemp(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*laserCloudMsg, *laserCloud);

    pcl::removeNaNFromPointCloud(*laserCloud, *laserCloud, indiceLet);
    indiceLet.clear();

    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setInputCloud(laserCloud);
    downSizeFilter.setLeafSize(voxel_size, voxel_size, voxel_size);
    downSizeFilter.filter(*laserCloud);
    pcl::removeNaNFromPointCloud(*laserCloud, *laserCloud, indiceLet);
    indiceLet.clear();
    pcl::transformPointCloud(*laserCloud, *laserCloudVisual, meas_TF);
    if (debug_flag == true)
    {

        sensor_msgs::msg::PointCloud2 laserCloudFeature2;
        pcl::toROSMsg(*laserCloudVisual, laserCloudFeature2);
        laserCloudFeature2.header.stamp = rclcpp::Time(timeLaserCloud,timeLaserCloudNano);
        laserCloudFeature2.header.frame_id = "map";
        pubLaserCloud->publish(laserCloudFeature2);

        pubSurroundMap();
    }

    state_meas(0) = state_TF(0,3);
    state_meas(1) = state_TF(1,3);
    state_meas(2) = state_TF(2,3);
    Eigen::Matrix3f meas_R = state_TF.block(0,0,3,3);

    Eigen::Quaternionf q(meas_R);
    tf2::Quaternion q_meas;
    q_meas.setW(q.w());
    q_meas.setX(q.x());
    q_meas.setY(q.y());
    q_meas.setZ(q.z());
    tf2::Matrix3x3 m2(q_meas);
    m2.getRPY(state_meas(3), state_meas(4), state_meas(5));
   
    // initializing
    if (!initSequence)
    {
        initSequence = true;
        RCLCPP_INFO(this->get_logger(), "Mapping initialization finished \n");
    }
    else
    {
        TicToc t_localization;
        Eigen::Matrix4f TF (Eigen::Matrix4f::Identity());
        Eigen::Matrix3f R;
        R =  Eigen::AngleAxisf(state_meas(5), Eigen::Vector3f::UnitZ())
            * Eigen::AngleAxisf(state_meas(4), Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(state_meas(3), Eigen::Vector3f::UnitX());

        TF.block(0,0,3,3) = R;
        TF(0,3) = state_meas(0); TF(1,3) = state_meas(1);  TF(2,3) = state_meas(2);


        pcl::PointCloud<PointType>::Ptr aligned_cloud(new pcl::PointCloud<PointType>());
        if (matching_mode == "NDT")
        {
            ndt.setInputSource(laserCloud);
            ndt.align(*aligned_cloud, TF);
            meas_TF = ndt.getFinalTransformation();
        }
        else if (matching_mode == "NDT_OMP")
        {
            ndtomp.setInputSource(laserCloud);
            ndtomp.align(*aligned_cloud, TF);
            meas_TF = ndtomp.getFinalTransformation();
        }
        Eigen::Matrix<double, 6, 6> hessian = ndtomp.getHessian();

        typedef Eigen::EigenSolver<Eigen::Matrix<double, 6, 6>> EigenSolver;
        EigenSolver es;
        Eigen::Matrix<double, 6, 6> hessian_inv = hessian.inverse();
        es.compute(hessian_inv);
        hessian_diag = hessian_inv.diagonal().cwiseAbs();
        // EigenSolver::EigenvalueType eigenvalues = es.eigenvalues();
        Eigen::VectorXcd eigenvalues = es.eigenvalues();

        Eigen::VectorXd abs_eigen = eigenvalues.cwiseAbs();

        std::complex<double> max_eigenvalue = eigenvalues(0);
        for (int i = 1; i < eigenvalues.size(); ++i) 
        {
            if (eigenvalues(i).real() > max_eigenvalue.real()) {

                max_eigenvalue = eigenvalues(i);
            }
        }

        double max_eigen = max_eigenvalue.real();
        double max_abs_eigen = abs_eigen.maxCoeff();

        if(std::isnan(meas_TF(0,3)) == true || std::isnan(meas_TF(1,3)) == true || std::isnan(meas_TF(2,3)) == true)
        {
            RCLCPP_INFO(this->get_logger(), "nan detected!! \n");            
        }
        else
        {
            pcl::transformPointCloud(*laserCloud, *laserCloud, meas_TF);
            matching_points_num = 0;
            int neighborThreshold = 1; // FIXME, we can make it as parameter, 1~7 based on DIRECT7
            if(matching_mode == "NDT_OMP") 
            {
                for(int k = 0; k < laserCloud->points.size(); k++) 
                {
                    std::vector<pclompm::VoxelGridCovariance<PointType>::LeafConstPtr> neighborhood;
                    int validGrid = ndtomp.getTargetGridPtr()->getNeighborhoodAtPoint7(laserCloud->points[k], neighborhood);
                    if(validGrid >= neighborThreshold) 
                    {
                        matching_points_num++;
                    }

                    kdtree->nearestKSearch(laserCloud->points[k], 1, pointSearchInd, pointSearchSqDis);
                    if (pointSearchSqDis[0] < 0.1)
                    {
                        laserCloudGlobal->points.push_back(laserCloud->points[k]);
                    }
                }
            }

            pcl::VoxelGrid<PointType> downSizeFilter;
            downSizeFilter.setInputCloud(laserCloudGlobal);
            downSizeFilter.setLeafSize(2.0, 2.0, 2.0);
            downSizeFilter.setMinimumPointsNumberPerVoxel(5);
            downSizeFilter.filter(*laserCloudGlobal);
            pcl::removeNaNFromPointCloud(*laserCloudGlobal, *laserCloudGlobal, indiceLet);
            indiceLet.clear();
            std::vector<Eigen::Vector3d> range_info;
            for (size_t k = 0; k < laserCloudGlobal->points.size(); k++)
            {
                if (laserCloudGlobal->points[k]._PointXYZINormal::normal_x == 1)
                {
                    continue;
                }
                double r = sqrt(pow((laserCloudGlobal->points[k].x - meas_TF(0, 3)), 2) + pow((laserCloudGlobal->points[k].y - meas_TF(1, 3)), 2) + pow((laserCloudGlobal->points[k].z - meas_TF(2, 3)), 2));
                if (r < 1.0)
                {
                    continue;
                }
                Eigen::Vector3d r_info;
                r_info(0) = (laserCloudGlobal->points[k].x - meas_TF(0, 3)) / r;
                r_info(1) = (laserCloudGlobal->points[k].y - meas_TF(1, 3)) / r;
                r_info(2) = (laserCloudGlobal->points[k].z - meas_TF(2, 3)) / r;
                range_info.push_back(r_info);
            }
            Eigen::MatrixXd AA(range_info.size(), 3);
            for (size_t p = 0; p < range_info.size(); p++)
            {
                AA(p, 0) = range_info[p](0);
                AA(p, 1) = range_info[p](1);
                AA(p, 2) = range_info[p](2);
            }
            
            Eigen::Matrix3d A_sq;
            Eigen::Matrix3d Q;
            A_sq = AA.transpose() * AA;
            Q = A_sq.inverse();
            pdop = sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));
            hdop = sqrt(Q(0, 0) + Q(1, 1));
            if (pdop == 0 || pdop > 10000 || std::isnan(pdop) == true)
            {
                hdop = 10000;
                pdop = 10000;
            }
            
            if (lidar_handler_on && pose_estimation_on) 
            {
                pubMeasurement();
                // eigenvalue threshold : 0.00006
                if (max_eigen < 0)
                {
                    hessian_Converged = true;
                    if (max_abs_eigen < 0.00006) 
                    {   
                        hessian_Status = true;
                        RCLCPP_INFO_STREAM(this->get_logger(), "\x1b[32m" "Good Map Matching Result." "\x1b[0m");
                    }
                    else
                    {
                        hessian_Status = false;
                        RCLCPP_INFO_STREAM(this->get_logger(), "\x1b[33m" "Map Matching Converged." "\x1b[0m");
                    }
                }                
                else 
                {
                    hessian_Status = false;
                    hessian_Converged = false;
                    RCLCPP_INFO_STREAM(this->get_logger(), "\x1b[31m" "Bad Map Matching Result." "\x1b[0m");
                }
            }
            wholeLocalizationProcessTime = t_localization.toc();
            matching_failed = false;
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<robotLocalization::ndtLocalization>());

    rclcpp::shutdown();

    return 0;
}
