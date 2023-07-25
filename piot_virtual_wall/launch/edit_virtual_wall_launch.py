import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition

def generate_launch_description():
	use_sim_time = LaunchConfiguration(
		'use_sim_time',
		default='false')
	param_dir = LaunchConfiguration(
		'params_file',
		default=os.path.join(
			get_package_share_directory('seokgyun_bringup'),
			'params',
			'nav2_params.yaml')) 
	map_dir = LaunchConfiguration(
		'map',
		default=os.path.expanduser(os.path.join(
			'~',
			'f1.yaml')))
	namespace = LaunchConfiguration(
		'namespace',
		default='')
	use_respawn = LaunchConfiguration(
		'use_respawn',
		default='False')
	log_level = LaunchConfiguration(
		'log_level',
		default='info')
	autostart = LaunchConfiguration(
		'autostart',
		default='True')
	use_composition = LaunchConfiguration(
		'use_composition',
		default='True')   
      
	remappings = [('/tf', 'tf'),
								('/tf_static', 'tf_static')]
    	
	rviz_config_dir = os.path.join(
		get_package_share_directory('piot_virtual_wall'),
		'rviz',
		'piot_virtual_wall.rviz')
       
	param_substitutions = {
    'use_sim_time': use_sim_time,
    'yaml_filename': map_dir}
        
	configured_params = RewrittenYaml(
		source_file=param_dir,
		root_key=namespace,
		param_rewrites=param_substitutions,
		convert_types=True)
   
	lifecycle_nodes = ['map_server']
               
	return LaunchDescription([
		Node(
			condition=IfCondition(use_composition), 
			name='nav2_container',
			package='rclcpp_components',
			executable='component_container_isolated',
			parameters=[configured_params, {'autostart': autostart}],
			arguments=['--ros-args', '--log-level', log_level],
			remappings=remappings,
			output='screen'
		),
   
		Node(
			package='nav2_map_server',
			executable='map_server',
			name='map_server',
			output='screen',
			respawn=use_respawn,
			respawn_delay=2.0,
			parameters=[configured_params],
			arguments=['--ros-args', '--log-level', log_level],
			remappings=remappings
		),

		Node(
			package='piot_virtual_wall',
			executable='piot_virtual_wall',
			name='piot_edit_virtual_wall_node',
			output='screen',
		),

		Node(
		package='nav2_lifecycle_manager',
		executable='lifecycle_manager',
		name='lifecycle_manager_localization',
		output='screen',
		arguments=['--ros-args', '--log-level', log_level],
		parameters=[{'use_sim_time': use_sim_time},
    						{'autostart': autostart},
								{'node_names': lifecycle_nodes}]),
        
   
		Node(
			package='rviz2',
			executable='rviz2',
			name='rviz2',
			arguments=['-d', rviz_config_dir],
			parameters=[{'use_sim_time': use_sim_time}],
			output='screen'),
	])
