#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='piot_converter',
			executable='piot_converter',
			name='piot_converter_node',
			output='screen',
		)
	])
