#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='seokgyun_teleop',
			executable='seokgyun_teleop',
			name='seokgyun_teleop_node',
			output='screen',
			prefix='xterm -e'
		)
	])
