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

    robot_localization_pkg_dir = LaunchConfiguration(
        'robot_localization_pkg_dir',
        default=os.path.join(get_package_share_directory('seokgyun_robot_localization'),'launch'))


    converter_pkg_dir = LaunchConfiguration(
        'converter_pkg_dir',
        default=os.path.join(get_package_share_directory('robot_converter'), 'launch'))


    return LaunchDescription([

        ## only lidar +imusou   ./i 

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([robot_localization_pkg_dir, '/robot_localization_launch.py']),
        # ),

        ## + gps

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_localization_pkg_dir, '/gps_robot_localization_launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([converter_pkg_dir, '/robot_converter_launch.py']),
        ),
    ])
