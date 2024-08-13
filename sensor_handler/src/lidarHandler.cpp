#include "lidarHandler.hpp"

lidarHandler::lidarHandler()
    : rclcpp::Node("lidarHandler")
    , qos_(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_services_default))
{
    Initialize();
}

lidarHandler::~lidarHandler()
{
}

int lidarHandler::Initialize()
{
    this->declare_parameter<double>("minimum_range", 0.3);
    this->declare_parameter<double>("l2b_roll", 0);
    this->declare_parameter<double>("l2b_pitch", 0);
    this->declare_parameter<double>("l2b_yaw", 0);
    this->declare_parameter<double>("l2b_x", 0);
    this->declare_parameter<double>("l2b_y", 0);
    this->declare_parameter<double>("l2b_z", 0);
    this->get_parameter("minimum_range",MINIMUM_RANGE);
    this->get_parameter("l2b_roll", l2b_roll);
    this->get_parameter("l2b_pitch", l2b_pitch);
    this->get_parameter("l2b_yaw", l2b_yaw);
    this->get_parameter("l2b_x", l2b_x);
    this->get_parameter("l2b_y", l2b_y);
    this->get_parameter("l2b_z", l2b_z);

    Eigen::Matrix3f R;
    L2B_TF.setIdentity();
    R = Eigen::AngleAxisf(deg2rad(l2b_roll), Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(deg2rad(l2b_pitch), Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(deg2rad(l2b_yaw), Eigen::Vector3f::UnitZ());
    L2B_TF.block(0,0,3,3) = R;

    bodyVelo.setZero();

    subDR = create_subscription<nav_msgs::msg::Odometry>(
        "dr/velo", qos_, std::bind(&lidarHandler::drHandler, this, std::placeholders::_1));
    subLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2>(
        "hesai/pandar", qos_, std::bind(&lidarHandler::laserCloudHandler, this, std::placeholders::_1));
    subResponse = create_subscription<std_msgs::msg::Bool>(
        "/response_signal_NL", qos_, std::bind(&lidarHandler::responseCallback, this, std::placeholders::_1));

    groundCoeff = create_publisher<nav_msgs::msg::Odometry>("groundCoeff", qos_);
    pubProcessingPoints = create_publisher<sensor_msgs::msg::PointCloud2>("lidar_points", qos_);
    pubDummyLidar = create_publisher<std_msgs::msg::Float32>("dummyLidar", qos_);
    pubReadyFlag = create_publisher<std_msgs::msg::Empty>("/ready_signal_LH", qos_);
    
    return 0;
}

template <typename PointT>
void lidarHandler::removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                            pcl::PointCloud<PointT> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (isnanl(cloud_in.points[i].x))        continue;
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;

        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

pcl::PointCloud<PointType>::Ptr lidarHandler::Extract_ground_Points(pcl::PointCloud<PointType>::Ptr laserPointCloud, pcl::PointCloud<PointType>::Ptr laserPlaneCloud)
{
    Eigen::Vector3d p0; //라이다 위치에서의 지면 평면의 좌표
    Eigen::Vector4d coeff_value;  //지면 평면의 계수

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // RANSAC 알고리즘을 이용한 지면 평면 계수 추출
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE); //RANSAC 모델 : Plane
    seg.setMethodType (pcl::SAC_RANSAC);  //평면 추출 방법 : RANSAC
    seg.setDistanceThreshold (0.1);
    seg.setInputCloud (laserPlaneCloud);  //입력 점 군
    try
    {
        // 전체 점 군 개수가 200개 이상일 경우 RANSAC으로 평면 계수 추출
        if (laserPlaneCloud->points.size() > 200)
        {
            seg.segment (*inliers, *coefficients);
        }
        //추출된 inlier가 30개 이하이면 error 메시지 throw
        if (inliers->indices.size() < 30)
        {
            //throw string("RANSAC Inlier is empty.");
        }
        coeff_value(0) = coefficients->values[0]; coeff_value(1) = coefficients->values[1]; coeff_value(2) = coefficients->values[2]; coeff_value(3) = coefficients->values[3];
    }
    catch (string error)
    {
        cout << error << endl;
        coeff_value(0) = 0; coeff_value(1) = 0; coeff_value(2) = 1; coeff_value(3) = -1;  //조건에 맞지 않을 경우 수평면 기준으로 계수 입력
    }

    p0(0) = 0;  p0(1) = 0;  p0(2) = -coeff_value(3)/coeff_value(2);

    nav_msgs::msg::Odometry ground_coeff_msg;
    ground_coeff_msg.header.stamp = rclcpp::Clock().now();
    ground_coeff_msg.pose.pose.orientation.x = coeff_value(0);
    ground_coeff_msg.pose.pose.orientation.y = coeff_value(1);
    ground_coeff_msg.pose.pose.orientation.z = coeff_value(2);
    ground_coeff_msg.pose.pose.orientation.w = coeff_value(3);

    groundCoeff->publish(ground_coeff_msg);


    pcl::KdTreeFLANN<PointType>::Ptr kdtree (new pcl::KdTreeFLANN<PointType>());  //kd-tree 객체 선언
    kdtree->setInputCloud(laserPointCloud); //kd tree 내 점 군 입력
    std::vector<int> Ind;
    std::vector<float> Dis;
    for (int p = 0; p < laserPointCloud->points.size(); p++)
    {
        double dist = fabs(coeff_value(0)*laserPointCloud->points[p].x+
                           coeff_value(1)*laserPointCloud->points[p].y+
                           coeff_value(2)*laserPointCloud->points[p].z+
                           coeff_value(3))
                           / sqrt(pow(coeff_value(0),2)+pow(coeff_value(1),2)+pow(coeff_value(2),2)); //지평면과 라이다 포인트 간의 거리

        double dist_thres = 0.01;
        double range = sqrt(pow((laserPointCloud->points[p].x-l2b_x),2)+pow((laserPointCloud->points[p].y-l2b_y),2));
        if (range > 1 && range < 4)   dist_thres = 0.02*(range-1.0)/3.0+0.01;
        else if (range >= 4)   dist_thres = 0.03;
        double plane_z = -1/coeff_value(2)*(coeff_value(0)*laserPointCloud->points[p].x+coeff_value(1)*laserPointCloud->points[p].y+coeff_value(3));
        if (laserPointCloud->points[p].z < plane_z || dist < dist_thres)
        {
            //PlaneCloud 중 가장 가까운 전체 라이다 점 군을 찾아 normal_x에 1 입력 (0 : 지면이 아닌 포인트, 1 : 지면에 해당하는 포인트)
            //kdtree->nearestKSearch(laserPlaneCloud->points[p], 1, Ind, Dis);
            laserPointCloud->points[p]._PointXYZINormal::normal_x = 1;
        }
    }
    return laserPointCloud;
}

void lidarHandler::responseCallback(const std_msgs::msg::Bool::SharedPtr rspsMsg) 
{
    if (rspsMsg->data) 
    {
        response_subscribed = true;
    }
}

void lidarHandler::laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
{
    if (!flag_published) 
    {

        pubReadyFlag->publish(std_msgs::msg::Empty());
        
        RCLCPP_INFO_ONCE(this->get_logger(), "\x1b[32m" "lidarHandler Ready." "\x1b[0m");
        
        if (response_subscribed) {

            flag_published = true;
        }
    }
    std_msgs::msg::Float32 dummyMsg;
    pubDummyLidar->publish(dummyMsg);

    TicToc t_lidar;
    pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

    int cloudSize = laserCloudIn.points.size();
    int scanID = 0;
    PointType point;
    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr laserPlaneCloud(new pcl::PointCloud<PointType>());

    std::vector<int> indices;
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);
    pcl::transformPointCloud(laserCloudIn, laserCloudIn, L2B_TF);

    cloudSize = laserCloudIn.points.size();
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                          laserCloudIn.points[cloudSize - 1].x) +
                   2 * M_PI;

    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }

    bool halfPassed = false;
    for (int i = 0; i < cloudSize; i++)
    {
        if (laserCloudIn.points[i].z < -0.25)  continue;
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        // scanID = laserCloudIn.points[i].ring;
        point.intensity = laserCloudIn.points[i].intensity;

        float ori = -atan2(point.y, point.x);
        if (!halfPassed)
        {
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }
        float relTime = (ori - startOri) / (endOri - startOri);
        point._PointXYZINormal::curvature = scanID + scanPeriod * relTime;

        double scale = (point._PointXYZINormal::curvature - int(point._PointXYZINormal::curvature)) / scanPeriod;
        if (scale > 1)
        {
            scale -= int(scale);
        }
        Eigen::Vector3f linearInc = bodyVelo * scanPeriod * scale;
        Eigen::Vector3f angInc = rpy_inc * scanPeriod * scale;
        Eigen::Matrix3f R;
        R = Eigen::AngleAxisf(deg2rad(angInc(0)), Eigen::Vector3f::UnitX())
          * Eigen::AngleAxisf(deg2rad(angInc(1)), Eigen::Vector3f::UnitY())
          * Eigen::AngleAxisf(deg2rad(angInc(2)), Eigen::Vector3f::UnitZ());
        pcl::PointXYZ tempPoint;
        tempPoint.x = R(0,0) * point.x + R(0,1) * point.y + R(0,2) * point.z + linearInc(0);
        tempPoint.y = R(1,0) * point.x + R(1,1) * point.y + R(1,2) * point.z + linearInc(1);
        tempPoint.z = R(2,0) * point.x + R(2,1) * point.y + R(2,2) * point.z + linearInc(2);
        point.x = tempPoint.x;  point.y = tempPoint.y;  point.z = tempPoint.z;

        laserCloud->points.push_back(point);
        float angle = rad2deg(atan(point.z / sqrt(point.x * point.x + point.y * point.y)));
        point._PointXYZINormal::normal_x = 0;
        if (angle <= 0 && point.x > 0 && point.x < (4.0) && std::fabs(point.y) < 1.5 && point.z < -0.03)
        {
            laserPlaneCloud->points.push_back(point);
        }
    }

    // Extract_ground_Points(laserCloud, laserPlaneCloud); //지면 추출 포인트 함수 (입력 : 전체 라이다 점 군, 수직 각도 및 높이로 필터링한 점 군)

    sensor_msgs::msg::PointCloud2 lidarPointsMsg;
    pcl::toROSMsg(*laserCloud, lidarPointsMsg);
    lidarPointsMsg.header.stamp = laserCloudMsg->header.stamp;
    lidarPointsMsg.header.frame_id = "lidar";
    pubProcessingPoints->publish(lidarPointsMsg);
}

void lidarHandler::drHandler(const nav_msgs::msg::Odometry::SharedPtr drMsg)
{
    bodyVelo(0) = drMsg->twist.twist.linear.x;
    bodyVelo(1) = drMsg->twist.twist.linear.y;
    rpy_inc(0) = drMsg->twist.twist.angular.x;
    rpy_inc(1) = drMsg->twist.twist.angular.y;
    rpy_inc(2) = drMsg->twist.twist.angular.z;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<lidarHandler>());

    return 0;
}
