#!/usr/bin/env python
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
argu1 =['/scan','/imu_sensing','/wheel/odomtry']
def generate_launch_description():
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=['ros2','bag','play','rosbag/navigation/navigation_0.db3', '--topics' , '/scan','/imu_sensing','/wheel/odometry','/imu' ,'/initialpose' ],
                output="screen"
            )
        ]

        
    )