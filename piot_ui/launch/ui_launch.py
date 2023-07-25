#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='piot_ui',
            executable='piot_ui_node',
            name='piot_ui_node',
            output='screen'
        )
    ])


