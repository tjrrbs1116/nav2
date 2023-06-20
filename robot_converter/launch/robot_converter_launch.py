#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='robot_converter',
			executable='robot_converter',
			name='robot_converter',
			output='screen',
		)
	])
