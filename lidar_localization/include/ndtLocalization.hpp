#ifndef LIDAR_LOCALIZATION_HPP_
#define LIDAR_LOCALIZATION_HPP_
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <thread>
#include <optional>
#include <iostream>
#include <filesystem>
#include "common.h"
#include "tic_toc.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/ndt.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/imu.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <std_msgs/msg/float32.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <geometry_msgs/msg/twist_stamped.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <tf2/transform_datatypes.h>
#include <eigen3/Eigen/Eigen>
// #include <ceres/ceres.h>
#include "tf2_ros/transform_broadcaster.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include "tf2/transform_storage.h"
#include "tf2/utils.h"
#include <pclompm/ndt_omp.h>
#include <pclompm/voxel_grid_covariance_omp.h>

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

namespace pcl {
template <typename PointSource, typename PointTarget>
class NormalDistributionsTransformExtension : public NormalDistributionsTransform<PointSource, PointTarget> {
public:
    inline VoxelGridCovariance<PointTarget>* getTargetGridPtr() {
        return &(this->target_cells_);
    }
};
} // namespace pcl

namespace robotLocalization{
class ndtLocalization
    : public rclcpp::Node
{
public:
    ndtLocalization();
    ~ndtLocalization();

    int Initialize();

    void pubSurroundMap();
    void pubMeasurement();
    
    void currStateHandler(const nav_msgs::msg::Odometry::SharedPtr state);
    void laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);
    void flagCallbackLH(const std_msgs::msg::Empty::SharedPtr flag);
    void flagCallbackPE(const std_msgs::msg::Empty::SharedPtr flag);

private:
    bool initSequence = false;
    
    pcl::NormalDistributionsTransformExtension<PointType, PointType> ndt;
    pclompm::NormalDistributionsTransform<PointType, PointType> ndtomp;
    
    double timeLaserCloud = 0.0;
    double timeLaserCloudNano = 0.0;
    const double scanPeriod = 0.1;

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree;
    double pdop, hdop;
    double init_x, init_y, init_z, init_roll, init_pitch, init_yaw;

    std::string map_directory;
    std::string matching_mode;

    bool systemInited = false;

    double l2b_roll, l2b_pitch, l2b_yaw, l2b_x, l2b_y, l2b_z;
    Eigen::Matrix4f L2B_TF;
    Eigen::Matrix4f state_TF;
    Eigen::Matrix4f meas_TF;
    Eigen::VectorXd hessian_diag;

    Eigen::VectorXd g_TM_offset;

    double voxel_size;

    float mapMatchingRange = 0;
    int matching_points_num = 0;
    int max_points_num = 0;
    int nearCnt = 0;    
    int iter_num;
    int thread_num;
    double map_voxel_size;
    double step_size;
    double epsilon;
    double resolution;

    bool matching_failed = false;

    pcl::PointCloud<pcl::PointXYZ> map_trajectory;

    Eigen::Vector3d initPos, initAtt;
    bool debug_flag = false;
    bool matching_flag = false;
    bool lidar_handler_on = false;
    bool pose_estimation_on = false;
    bool hessian_Converged = false;
    bool hessian_Status = false;

    // Publisher / Subscription
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubWallPoints;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubResponse;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubMeasurementInfo;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomCenterOfGravity;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubDummyMeas;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr subLaserCloud;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr subState;
    rclcpp::Subscription<std_msgs::msg::Empty>::ConstSharedPtr subReadyFlagLH;
    rclcpp::Subscription<std_msgs::msg::Empty>::ConstSharedPtr subReadyFlagPE;
    
    double wholeLocalizationProcessTime;

    nav_msgs::msg::Path measPath;

    pcl::PointCloud<PointType>::Ptr laserCloudMap;
    pcl::PointCloud<PointType>::Ptr laserCloudVisMap;
    pcl::PointCloud<PointType>::Ptr laserCloud;
    pcl::PointCloud<PointType>::Ptr laserCloudGlobal;

    std::vector<int> indiceLet;

    Eigen::VectorXd state_meas;

    rclcpp::QoS qos_;
    rclcpp::QoS qos_q1;
    
    geometry_msgs::msg::TransformStamped transf_;

    bool proc_finish_flg;

};
}
#endif // LIDAR_LOCALIZATION_HPP_
