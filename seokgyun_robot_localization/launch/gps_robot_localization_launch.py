import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# def generate_launch_description():
#     use_sim_time = LaunchConfiguration('use_sim_time', default='false')
#     config_dir = LaunchConfiguration(
#         'config_file',
#         default=os.path.join(
#             get_package_share_directory('piot_robot_localization'),
#             'config',
#             'ekf.yaml'))

#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'config_file',
#             default_value=config_dir,
#             description='Full path to config file to load'),

#         DeclareLaunchArgument(
#             'use_sim_time',
#             default_value='false',
#             description='Use simulation (Gazebo) clock if true'),

#         Node(
#             package='robot_localization',
#             executable='ekf_node',
#             name='ekf_filter_node',
#             parameters=[config_dir, {'use_sim_time': use_sim_time}],
#             output='screen',
#             remappings=[('/odometry/filtered','/odom')]),
#     ])

##with gps
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    config_dir = LaunchConfiguration(
        'config_file',
        default=os.path.join(
            get_package_share_directory('seokgyun_robot_localization'),
            'config',
            'odom_test.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_dir,
            description='Full path to config file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Node(
        #      package='robot_localization',
        #      executable='ekf_node',
        #      name='ekf_filter_node_map',
        #      parameters=[config_dir, {'use_sim_time': use_sim_time}],
        #      output='screen',
        #      remappings=[('/odometry/filtered','/odom/global'),
        #                  ('/set_pose', '/initialpose')]),

        # Node(
        #      package='robot_localization',
        #      executable='ekf_node',
        #      name='ekf_filter_node_map_gps_amcl',
        #      parameters=[config_dir, {'use_sim_time': use_sim_time}],
        #      output='screen',
        #      remappings=[('/odometry/filtered','/odom/global_sum'),
        #                  ('/set_pose', '/initialpose')]),


        # Node(
        #     package='robot_localization',
        #     executable='navsat_transform_node',
        #     name='navsat_transform',
        #     parameters=[config_dir, {'use_sim_time': use_sim_time}],
        #     output='screen',
        #     remappings=[('/odom','/odom'),
        #                 ('/imu/data', '/imu_sensing'),]),


        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[config_dir, {'use_sim_time': use_sim_time}],
            output='screen',
            remappings=[('/odometry/filtered','/odom')]),


        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node_test1',
        #     parameters=[config_dir, {'use_sim_time': use_sim_time}],
        #     output='screen',
        #     remappings=[('/odometry/filtered','/odom_ekf_debug')]),

        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node_test2',
        #     parameters=[config_dir, {'use_sim_time': use_sim_time}],
        #     output='screen',
        #     remappings=[('/odometry/filtered','/odom_test2')]),


    ])
