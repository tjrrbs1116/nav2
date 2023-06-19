#!/usr/bin/env python3
import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros

# def generate_launch_description():
#     pkg_share = launch_ros.substitutions.FindPackageShare(package='seokgyun_description').find('seokgyun_description')
#     default_model_path = os.path.join(pkg_share, 'urdf/piot.urdf')
#     use_sim_time = LaunchConfiguration('use_sim_time', default='false')
#     urdf_file_name = 'piot.urdf'

#     print("urdf_file_name : {}".format(urdf_file_name))

#     urdf = os.path.join(
#         get_package_share_directory('seokgyun_description'),
#         'urdf',
#         urdf_file_name)

#     # Major refactor of the robot_state_publisher
#     # Reference page: https://github.com/ros2/demos/pull/426
#     with open(urdf, 'r') as infp:
#         robot_desc = infp.read()

#     rsp_params = {'robot_description': robot_desc}

#     # print (robot_desc) # Printing urdf information.

#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'use_sim_time',
#             default_value='false',
#             description='Use simulation (Gazebo) clock if true'),
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             output='screen',
#             parameters=[rsp_params, {'use_sim_time': use_sim_time}],
#             arguments=[urdf]),
    
#         Node(
#             package='joint_state_publisher',
#             executable='joint_state_publisher',
#             name='joint_state_publisher'
#         # condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
#             )

#     ])

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='seokgyun_description').find('seokgyun_description')
    default_model_path = os.path.join(pkg_share, 'urdf/piot.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/gui.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        # condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        # condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
        # rviz_node
    ])
