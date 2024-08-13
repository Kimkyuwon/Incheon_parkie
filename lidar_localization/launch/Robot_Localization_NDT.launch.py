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

    map_path = os.path.join(
        get_package_share_directory('lidar_localization'), 'map', 'PointCloud_AF_TM_Map.pcd')

    rviz_path = os.path.join(
        get_package_share_directory('lidar_localization'), 'rviz', 'goalie_localization.rviz')

    lidar_localization_config = launch.substitutions.LaunchConfiguration(
        'lidar_localization_config',
        default = os.path.join(
            get_package_share_directory('lidar_localization'),
            'config', 'lidar_localization_config.yaml'))
    
    pose_estimation_config = launch.substitutions.LaunchConfiguration(
        'pose_estimation_config',
        default = os.path.join(
            get_package_share_directory('lidar_localization'),
            'config', 'pose_estimation_config.yaml'))

    sensor_handler_config = launch.substitutions.LaunchConfiguration(
        'sensor_handler_config',
        default = os.path.join(
            get_package_share_directory('lidar_localization'),
            'config', 'sensor_handler_config.yaml'))
    
    sensor_checker = launch_ros.actions.LifecycleNode(
        name='sensorChecker',
        package='goalie_sensor_handler',
        executable='sensorChecker',
        parameters=[{}],
        output='screen',
        namespace='')

    sensorErrorHandler = launch_ros.actions.LifecycleNode(
        name='sensorErrorHandler',
        package='goalie_sensor_handler',
        executable='sensorErrorHandler',
        parameters=[{}],
        output='screen',
        namespace='')

    DR_Handler = launch_ros.actions.LifecycleNode(
        name='DR_Handler',
        package='goalie_sensor_handler',
        executable='DR_Handler',
        output='screen',
        namespace='')

    lidarHandler = launch_ros.actions.LifecycleNode(
        name='lidarHandler',
        package='goalie_sensor_handler',
        executable='lidarHandler',
        parameters=[sensor_handler_config],
        output='screen',
        namespace='')

    gnssHandler = launch_ros.actions.LifecycleNode(
        name='gnssHandler',
        package='goalie_sensor_handler',
        executable='gnssHandler',
        parameters=[],
        output='screen',
        namespace='')
    
    ndtLocalization = launch_ros.actions.LifecycleNode(
        name='ndtLocalization',
        package='lidar_localization',
        executable='ndtLocalization',
        parameters=[lidar_localization_config,
                    {"map_directory": map_path}],
        output='screen',
        namespace='')

    poseEstimation = launch_ros.actions.LifecycleNode(
        name='poseEstimation',
        package='pose_estimation',
        executable='poseEstimation',
        parameters=[pose_estimation_config],
        output='screen',
        namespace='')

    rviz2 = launch_ros.actions.LifecycleNode(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        arguments=['-d' + str(rviz_path)],
        output='screen',
        namespace='')

    ld.add_action(DR_Handler)
    ld.add_action(lidarHandler)
    ld.add_action(gnssHandler)
    ld.add_action(ndtLocalization)
    ld.add_action(poseEstimation)
    ld.add_action(sensor_checker)
    ld.add_action(sensorErrorHandler)
    ld.add_action(rviz2)

    return ld