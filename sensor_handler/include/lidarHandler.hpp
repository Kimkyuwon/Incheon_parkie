#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include "common.h"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/transform_datatypes.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/impl/utils.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std;

class lidarHandler
    : public rclcpp::Node{
public:
    lidarHandler();
    ~lidarHandler();
    int Initialize();

    template <typename PointT>
    void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres);
    pcl::PointCloud<PointType>::Ptr Extract_ground_Points(pcl::PointCloud<PointType>::Ptr laserCloud, pcl::PointCloud<PointType>::Ptr laserPointCloud);

    void drHandler(const nav_msgs::msg::Odometry::SharedPtr drMsg);
    void laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);
    void responseCallback(const std_msgs::msg::Bool::SharedPtr rspsMsg);

private:
    const double scanPeriod = 0.1;
    double l2b_roll, l2b_pitch, l2b_yaw, l2b_x, l2b_y, l2b_z;
    Eigen::Matrix4f L2B_TF;
    Eigen::Vector3f bodyVelo, rpy_inc;
    bool flag_published = false;
    bool response_subscribed = false;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pubReadyFlag;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubProcessingPoints;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubDummyLidar;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr subDR;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr subLaserCloud;
    rclcpp::Subscription<std_msgs::msg::Bool>::ConstSharedPtr subResponse;

    // add
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr groundCoeff;

    double MINIMUM_RANGE;
    rclcpp::QoS qos_;
};
