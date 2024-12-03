#include <cstdio>
#include <fstream>
#include <math.h>
#include <vector>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>
#include <filesystem>
#include <mutex>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/transform_datatypes.h>
#include "tf2_ros/transform_broadcaster.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include "tf2/transform_storage.h"
#include "tf2/utils.h"

#define COUNT_LIMIT 100
using namespace std;

class Evaluation_node : public rclcpp::Node
{
  public:
    Evaluation_node()
    : Node("evaluation_node")
    {
      sub_count = 0;
      bias_x = 0.0;
      bias_y = 0.0;
      bias_yaw = 0.0;
      limit_over_flag = false;

      pose_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "pose_COG_evaluation", 10, std::bind(&Evaluation_node::pose_callback, this, std::placeholders::_1));

      evaluation_pose_publisher = create_publisher<nav_msgs::msg::Odometry>("pose_COG_evaluation_to_LaserTracker", 10);

    }

  private:
    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if(limit_over_flag == false)
        {
          if(sub_count < COUNT_LIMIT)
          {
            Eigen::VectorXd g_state_cog_1(3);
            tf2::Quaternion q_cog_1;

            q_cog_1.setW(msg->pose.pose.orientation.w);
            q_cog_1.setX(msg->pose.pose.orientation.x);
            q_cog_1.setY(msg->pose.pose.orientation.y);
            q_cog_1.setZ(msg->pose.pose.orientation.z);
            tf2::Matrix3x3 m_1(q_cog_1);
            m_1.getRPY(g_state_cog_1(0), g_state_cog_1(1), g_state_cog_1(2));

            bias_x = bias_x + msg->pose.pose.position.x;
            bias_y = bias_y + msg->pose.pose.position.y;
            bias_yaw = bias_yaw + g_state_cog_1(2);

            sub_count = sub_count + 1;
            return;
          }
          else
          {
            bias_x = bias_x/(float)sub_count;
            bias_y = bias_y/(float)sub_count;
            bias_yaw = bias_yaw/(float)sub_count;
            limit_over_flag = true;
            RCLCPP_INFO_STREAM(this->get_logger(), "\x1b[32m" "Evaluation Pose is Ready." "\x1b[0m");
            cout << "[bias   x] : " << bias_x << endl;
            cout << "[bias   y] : " << bias_y << endl;
            cout << "[bias yaw] : " << bias_yaw << endl;

          }
      }
      else
      {
        nav_msgs::msg::Odometry robotPose_evaluation;
        robotPose_evaluation = *msg;
        
        Eigen::VectorXd g_state_cog_2(3);
        tf2::Quaternion q_cog_2;

        q_cog_2.setW(msg->pose.pose.orientation.w);
        q_cog_2.setX(msg->pose.pose.orientation.x);
        q_cog_2.setY(msg->pose.pose.orientation.y);
        q_cog_2.setZ(msg->pose.pose.orientation.z);
        tf2::Matrix3x3 m_2(q_cog_2);
        m_2.getRPY(g_state_cog_2(0), g_state_cog_2(1), g_state_cog_2(2));

        Eigen::Matrix4d global_TF_1(Eigen::Matrix4d::Identity());
        Eigen::Matrix3d global_R_1;
        global_R_1 =  Eigen::AngleAxisd(g_state_cog_2(2), Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(g_state_cog_2(1), Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(g_state_cog_2(0), Eigen::Vector3d::UnitX());
        global_TF_1.block(0,0,3,3) = global_R_1;
        global_TF_1(0,3) = msg->pose.pose.position.x;
        global_TF_1(1,3) = msg->pose.pose.position.y;
        global_TF_1(2,3) = msg->pose.pose.position.z;

        Eigen::Matrix4d global_TF_2(Eigen::Matrix4d::Identity());
        Eigen::Matrix3d global_R_2;
        global_R_2 =  Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
        global_TF_2.block(0,0,3,3) = global_R_2;
        global_TF_2(0,3) = -bias_x;
        global_TF_2(1,3) = -bias_y;
        global_TF_2(2,3) = 0.0;
        
        Eigen::Matrix4d global_TF_3(Eigen::Matrix4d::Identity());
        Eigen::Matrix3d global_R_3;
        global_R_3 =  Eigen::AngleAxisd(-bias_yaw, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
        global_TF_3.block(0,0,3,3) = global_R_3;
        global_TF_3(0,3) = 0.0;
        global_TF_3(1,3) = 0.0;
        global_TF_3(2,3) = 0.0;

        Eigen::Matrix4d global_TF_4(Eigen::Matrix4d::Identity());
        global_TF_4 = global_TF_3 * global_TF_2 * global_TF_1;

        Eigen::Matrix3d cog_R_evaluation;
        cog_R_evaluation = global_TF_4.block(0,0,3,3);
        Eigen::VectorXd g_state_cog_evaluation(6);

        Eigen::Quaterniond q_evaluation(cog_R_evaluation);
        tf2::Quaternion q_cog_evaluation;
        q_cog_evaluation.setW(q_evaluation.w());
        q_cog_evaluation.setX(q_evaluation.x());
        q_cog_evaluation.setY(q_evaluation.y());
        q_cog_evaluation.setZ(q_evaluation.z());

        robotPose_evaluation.pose.pose.position.x = global_TF_4(0,3);
        robotPose_evaluation.pose.pose.position.y = global_TF_4(1,3);
        robotPose_evaluation.pose.pose.orientation.x = q_cog_evaluation.x(); 
        robotPose_evaluation.pose.pose.orientation.y = q_cog_evaluation.y();
        robotPose_evaluation.pose.pose.orientation.z = q_cog_evaluation.z();
        robotPose_evaluation.pose.pose.orientation.w = q_cog_evaluation.w();

        evaluation_pose_publisher->publish(robotPose_evaluation);
      }
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr evaluation_pose_publisher;

    float bias_x;
    float bias_y;
    float bias_yaw;
    int sub_count;
    bool limit_over_flag;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Evaluation_node>());
  rclcpp::shutdown();
  return 0;
}