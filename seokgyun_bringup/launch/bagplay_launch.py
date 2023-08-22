#!/usr/bin/env python
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import ThisLaunchFileDir

import os
argu1 =['/scan','/imu_sensing','/wheel/odomtry']

bringup_dir = get_package_share_directory('seokgyun_bringup')
launch_dir = os.path.join(bringup_dir, 'launch')

#rosbag2_2023_07_26
def generate_launch_description():
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=['ros2','bag','play','hangang_waypoint_amcl_module2/hangang_waypoint_amcl_module2_0.db3', '--topics' , '/scan','/imu_sensing','/ctrl_fb','/imu' ,'/initialpose' ],
                output="screen"
            ),

            IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/debug_map_server_launch.py']),
        ),

            IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/rviz_launch.py']),
        ),

            IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/local_localization_launch.py']),
        ),


            IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/state_publisher_launch.py']),
        ),
        ]

        
    )