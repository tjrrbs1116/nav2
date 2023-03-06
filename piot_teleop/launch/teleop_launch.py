#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='piot_teleop',
			executable='piot_teleop',
			name='piot_teleop_node',
			output='screen',
			prefix='xterm -e'
		)
	])
