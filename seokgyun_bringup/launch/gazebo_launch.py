# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration , PythonExpression
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.conditions import IfCondition

def generate_launch_description():
    bringup_dir = get_package_share_directory('seokgyun_bringup')
    description_dir = get_package_share_directory('seokgyun_description')
    launch_dir = os.path.join(bringup_dir, 'launch')
    
    localization_config_dir = LaunchConfiguration(
        'config_file',
        default=os.path.join(
            get_package_share_directory('piot_robot_localization'),
            'config',
            'sim_ekf.yaml'))
    

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_dqn_stage4.world'
    )
    namespace = LaunchConfiguration('namespace', default ='')

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
#------------------simulation robot ---------------------------------#
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')
    pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
            'y': LaunchConfiguration('y_pose', default='-0.50'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
    
    urdf = os.path.join(description_dir, 'urdf', 'piot.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    rsp_params = {'robot_description': robot_description}

    
#--------------------------------------------------------------------#

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot3_waffle', #turtlebot3_waffle
        description='name of the robot')
    
    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(bringup_dir, 'worlds', 'waffle.model'),
        description='Full path to robot sdf file to spawn the robot in gazebo')
    
    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-file', robot_sdf,
            '-robot_namespace', namespace,
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])
    
    

    
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    ) 

    start_gazebo_client_cmd = ExecuteProcess(

        cmd=['gzclient'],
        cwd=[launch_dir], output='screen')
    

    robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[rsp_params, {'use_sim_time': use_sim_time}],
            arguments=[urdf])


    # joint_state_publisher_node = Node(
    #         package='joint_state_publisher',
    #         executable='joint_state_publisher',
    #         name='joint_state_publisher',
    #         parameters=[rsp_params, {'use_sim_time': use_sim_time}],
    #         arguments=[urdf])
    #         # condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))

    localization_cmd2 =  Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[localization_config_dir, {'use_sim_time': use_sim_time}],
            output='screen',
            )#remappings=[('/odometry/filtered','/odom')])

    ld = LaunchDescription()

    #ld.add_action(declare_world_cmd)

    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(start_gazebo_spawner_cmd)
    ld.add_action(robot_state_publisher_cmd)
    #ld.add_action(start_gazebo_server_cmd)
    #ld.add_action(joint_state_publisher_node)
    ld.add_action(gzserver_cmd)
    ld.add_action(start_gazebo_client_cmd)
    # ld.add_action(localization_cmd2)
    return ld

