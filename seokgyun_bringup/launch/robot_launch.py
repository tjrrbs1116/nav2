#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    
    can_control_pkg_dir = LaunchConfiguration(
        'can_control_pkg_dir',
        default=os.path.join(get_package_share_directory('piot_can_control'), 'launch'))

    converter_pkg_dir = LaunchConfiguration(
        'converter_pkg_dir',
        default=os.path.join(get_package_share_directory('piot_converter')))
    
    robot_localization_pkg_dir = LaunchConfiguration(
        'robot_localization_pkg_dir',
        default=os.path.join(get_package_share_directory('piot_robot_localization'),'launch'))
     
    lidar_pkg_dir = LaunchConfiguration(
        'lidar_pkg_dir',
        default=os.path.join(get_package_share_directory('rplidar_ros2'), 'launch'))

    imu_pkg_dir = LaunchConfiguration(
        'imu_pkg_dir',
        default=os.path.join(get_package_share_directory('witmotion_ros'), 'launch'))
        
    depth_camera_pkg_dir = LaunchConfiguration(
        'depth_camera_pkg_dir',
        default=os.path.join(get_package_share_directory('astra_camera'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/state_publisher_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([can_control_pkg_dir, '/can_control_launch.py']),
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([converter_pkg_dir, '/converter_launch.py']),
        ),    

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_localization_pkg_dir, '/robot_localization_launch.py']),
        ),   

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, '/rplidar_s2_launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([imu_pkg_dir, '/witmotion.py']),
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([depth_camera_pkg_dir, '/dabai.launch.py']),
        ),
    ])
