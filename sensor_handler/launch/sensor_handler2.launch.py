import os

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = launch.LaunchDescription()

    rviz_path = os.path.join(
        get_package_share_directory('sensor_handler'), 'rviz', 'multilidar.rviz')

    DR_Handler = launch_ros.actions.LifecycleNode(
        name='DR_Handler',
        package='sensor_handler',
        executable='DR_Handler',
        output='screen')
    
    gnssHandler = launch_ros.actions.LifecycleNode(
    name='gnssHandler',
    package='sensor_handler',
    executable='gnssHandler',
    output='screen')

    lidarHandler = launch_ros.actions.LifecycleNode(
        name='lidarHandler',
        package='sensor_handler',
        executable='lidarHandler',
        parameters=[{
        "minimum_range": 0.5,
        "scan_line": 64,
        "lidar_type": "QT64",
        "lidar_topic": "hesai/pandar",
        "l2b_roll": 0.7,
        "l2b_pitch": 0.0,
        "l2b_yaw": 90.0,
        "l2b_x": 0.0,
        "l2b_y": 0.0,
        "l2b_z": 0.0}],
        output='screen')

    rviz2 = launch_ros.actions.LifecycleNode(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        arguments=['-d' + str(rviz_path)],
        output='screen')

    ld.add_action(DR_Handler)
    ld.add_action(gnssHandler)
    ld.add_action(lidarHandler)
    # ld.add_action(rviz2)

    return ld

