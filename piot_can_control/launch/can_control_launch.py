#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='piot_can_control',
			executable='piot_can_control_node',
			name='piot_can_control_node',
			output='screen'
		)
	])
	    
    
