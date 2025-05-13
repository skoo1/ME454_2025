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

import os
from launch import LaunchDescription
from launch import LaunchDescriptionEntity
from launch.actions import IncludeLaunchDescription , DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, Command, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():


	gazebo_models_path = 'models'
	gazebo_resource_path = 'rsc'
	package_name = 'vibration'
	rviz_config_file_path = 'rviz/urdf_gazebo_config.rviz'

	four_bar_linkage_sdf_file_path = 'rsc/urdf/spring_damper_single.sdf'
	world_file_path = 'worlds/vibration_test.world'


	gui = LaunchConfiguration('gui')
	headless = LaunchConfiguration('headless')
	namespace = LaunchConfiguration('namespace')
	rviz_config_file = LaunchConfiguration('rviz_config_file')
	use_namespace = LaunchConfiguration('use_namespace')
	use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
	use_rviz = LaunchConfiguration('use_rviz')
	use_sim_time = LaunchConfiguration('use_sim_time')
	use_simulator = LaunchConfiguration('use_simulator')
	world = LaunchConfiguration('world')

	pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
	pkg_share = FindPackageShare(package=package_name).find(package_name)
	default_model_path = os.path.join(pkg_share, four_bar_linkage_sdf_file_path)

	default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)
	world_path = os.path.join(pkg_share, world_file_path)
	gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
	gazebo_resource_path = os.path.join(pkg_share, gazebo_resource_path)


	os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

	# Declare the launch arguments   

	declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
	name='gui',
	default_value='True',
	description='Flag to enable joint_state_publisher_gui')

	declare_namespace_cmd = DeclareLaunchArgument(
	name='namespace',
	default_value='',
	description='Top-level namespace')
	
	declare_use_namespace_cmd = DeclareLaunchArgument(
	name='use_namespace',
	default_value='false',
	description='Whether to apply a namespace to the navigation stack')
	
	declare_rviz_config_file_cmd = DeclareLaunchArgument(
	name='rviz_config_file',
	default_value=default_rviz_config_path,
	description='Full path to the RVIZ config file to use')
	
	declare_simulator_cmd = DeclareLaunchArgument(
	name='headless',
	default_value='False',
	description='Whether to execute gzclient')
	
	declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
	name='use_robot_state_pub',
	default_value='True',
	description='Whether to start the robot state publisher')
	
	declare_use_rviz_cmd = DeclareLaunchArgument(
	name='use_rviz',
	default_value='True',
	description='Whether to start RVIZ')
	
	declare_use_sim_time_cmd = DeclareLaunchArgument(
	name='use_sim_time',
	default_value='true',
	description='Use simulation (Gazebo) clock if true')
	
	declare_use_simulator_cmd = DeclareLaunchArgument(
	name='use_simulator',
	default_value='True',
	description='Whether to start the simulator')
	
	declare_world_cmd = DeclareLaunchArgument(
	name='world',
	default_value=world_path,
	description='Full path to the world model file to load')
	
	start_joint_state_publisher_cmd = Node(
	package='joint_state_publisher',
	executable='joint_state_publisher',
	name='joint_state_publisher',
	condition=UnlessCondition(gui))
	
	start_gazebo_server_cmd = IncludeLaunchDescription(
	PythonLaunchDescriptionSource([os.path.join("/opt/ros/humble/share/gazebo_ros", 'launch', 'gzserver.launch.py')]),
	condition=IfCondition(use_simulator), 
	launch_arguments={'world': world,"verbose": "true", "pause": "true","use_sim_time":"true"}.items())
	
	start_gazebo_client_cmd = IncludeLaunchDescription(
	PythonLaunchDescriptionSource([os.path.join("/opt/ros/humble/share/gazebo_ros", 'launch', 'gzclient.launch.py')]),
	condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
 
	spawn_model_cmd = Node(
	package='gazebo_ros', 
	executable='spawn_entity.py',
	arguments=['-entity', 'spring_damper_system', 
				'-file', default_model_path,
				'-x', '0.0',
				'-y', '0.0',
				'-z', '0.15',
				'-R', '0.0',
				'-P', '0.0',
				'-Y', '0.0'
			],
	output='screen')

	ld = LaunchDescription()

	ld.add_action(declare_use_joint_state_publisher_cmd)
	ld.add_action(declare_namespace_cmd)
	ld.add_action(declare_use_namespace_cmd)
	ld.add_action(declare_rviz_config_file_cmd)
	ld.add_action(declare_simulator_cmd)
	ld.add_action(declare_use_robot_state_pub_cmd) 
	ld.add_action(declare_use_rviz_cmd) 
	ld.add_action(declare_use_sim_time_cmd)
	ld.add_action(declare_use_simulator_cmd)
	ld.add_action(declare_world_cmd)
	ld.add_action(start_gazebo_server_cmd)
	ld.add_action(start_gazebo_client_cmd)
	ld.add_action(spawn_model_cmd)
 
	return ld


