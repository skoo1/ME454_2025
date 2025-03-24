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
	package_name = 'bowling'
	robot_name_in_model = 'bowling_pin'
	rviz_config_file_path = 'rviz/urdf_gazebo_config.rviz'

	# Declare urdf file name to load
	pin_model_file_name = 'rsc/urdf/pin.urdf'
	ball_model_file_name = 'rsc/urdf/bowl.urdf'
	
	world_file_path = 'worlds/bowling.world'

	spawn_x_val_ball = '15.0'
	spawn_y_val_ball = '0.0'
	spawn_z_val_ball = '1.001'
	spawn_yaw_val_ball = '0.00'

	spawn_x_val_pin1 = '2.0'
	spawn_y_val_pin1 = '0.0'
	spawn_z_val_pin1 = '0.005'
	spawn_roll_val_pin1 = '1.57'
 
	spawn_x_val_pin2 = '1.0'
	spawn_y_val_pin2 = '1.0'
	spawn_z_val_pin2 = '0.005'
	spawn_roll_val_pin2 = '1.57'

	spawn_x_val_pin3 = '1.0'
	spawn_y_val_pin3 = '-1.0'
	spawn_z_val_pin3 = '0.005'
	spawn_roll_val_pin3 = '1.57'

	spawn_x_val_pin4 = '0.0'
	spawn_y_val_pin4 = '0.0'
	spawn_z_val_pin4 = '0.005'
	spawn_roll_val_pin4 = '1.57'

	spawn_x_val_pin5 = '0.0'
	spawn_y_val_pin5 = '-2.0'
	spawn_z_val_pin5 = '0.005'
	spawn_roll_val_pin5 = '1.57'

	spawn_x_val_pin6 = '0.0'
	spawn_y_val_pin6 = '2.0'
	spawn_z_val_pin6 = '0.005'
	spawn_roll_val_pin6 = '1.57'

	gui = LaunchConfiguration('gui')
	headless = LaunchConfiguration('headless')
	namespace = LaunchConfiguration('namespace')
	rviz_config_file = LaunchConfiguration('rviz_config_file')
	urdf_model = LaunchConfiguration('urdf_model')
	urdf_model_pin = LaunchConfiguration('urdf_model')
	use_namespace = LaunchConfiguration('use_namespace')
	use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
	use_rviz = LaunchConfiguration('use_rviz')
	use_sim_time = LaunchConfiguration('use_sim_time')
	use_simulator = LaunchConfiguration('use_simulator')
	world = LaunchConfiguration('world')

	pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
	pkg_share = FindPackageShare(package=package_name).find(package_name)
	default_urdf_pin_model_path = os.path.join(pkg_share, pin_model_file_name)
	default_urdf_ball_model_path= os.path.join(pkg_share, ball_model_file_name)



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
	
	declare_urdf_model_path_cmd = DeclareLaunchArgument(
	name='urdf_model', 
	default_value=default_urdf_pin_model_path, 
	description='Absolute path to robot urdf file')
	
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
 
	declare_gz_cmd = DeclareLaunchArgument(
	name='world',
	default_value=world_path,
	description='Full path to the world model file to load')
 


	start_robot_state_publisher_cmd = Node(
	package='robot_state_publisher',
	executable='robot_state_publisher',
	parameters=[{'robot_description': Command(['xacro ', urdf_model])}])
	
	start_joint_state_publisher_cmd = Node(
	package='joint_state_publisher',
	executable='joint_state_publisher',
	name='joint_state_publisher',
	condition=UnlessCondition(gui))

 
	start_gazebo_server_cmd = IncludeLaunchDescription(
	PythonLaunchDescriptionSource([os.path.join("/opt/ros/humble/share/gazebo_ros", 'launch', 'gzserver.launch.py')]),
	condition=IfCondition(use_simulator),
	launch_arguments={'world': world,"verbose": "true"}.items())
	
	start_gazebo_client_cmd = IncludeLaunchDescription(
	PythonLaunchDescriptionSource([os.path.join("/opt/ros/humble/share/gazebo_ros", 'launch', 'gzclient.launch.py')]),
	condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

	spawn_ball_cmd = Node(
	package='gazebo_ros', 
	executable='spawn_entity.py',
	arguments=['-entity', 'bowl', 
				'-file', default_urdf_ball_model_path,
					'-x', spawn_x_val_ball,
					'-y', spawn_y_val_ball,
					'-z', spawn_z_val_ball,
					'-Y', spawn_yaw_val_ball],
					output='screen')
	
	spawn_pin1_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', 'pin1', 
                '-topic', 'robot_description',
					'-x', spawn_x_val_pin1,
					'-y', spawn_y_val_pin1,
					'-z', spawn_z_val_pin1,
					'-R', spawn_roll_val_pin1],
                    output='screen')
 
	spawn_pin2_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', 'pin2', 
                '-topic', 'robot_description',
					'-x', spawn_x_val_pin2,
					'-y', spawn_y_val_pin2,
					'-z', spawn_z_val_pin2,
					'-R', spawn_roll_val_pin2],
                    output='screen')
  
	spawn_pin3_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', 'pin3', 
                '-topic', 'robot_description',
					'-x', spawn_x_val_pin3,
					'-y', spawn_y_val_pin3,
					'-z', spawn_z_val_pin3,
					'-R', spawn_roll_val_pin3],
                    output='screen')
   
	spawn_pin4_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', 'pin4', 
                '-topic', 'robot_description',
					'-x', spawn_x_val_pin4,
					'-y', spawn_y_val_pin4,
					'-z', spawn_z_val_pin4,
					'-R', spawn_roll_val_pin4],
                    output='screen')
    
	spawn_pin5_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', 'pin5', 
                '-topic', 'robot_description',
					'-x', spawn_x_val_pin5,
					'-y', spawn_y_val_pin5,
					'-z', spawn_z_val_pin5,
					'-R', spawn_roll_val_pin5],
                    output='screen')
     
	spawn_pin6_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', 'pin6', 
                '-topic', 'robot_description',
					'-x', spawn_x_val_pin6,
					'-y', spawn_y_val_pin6,
					'-z', spawn_z_val_pin6,
					'-R', spawn_roll_val_pin6],
                    output='screen')
 

	ld = LaunchDescription()

	ld.add_action(declare_simulator_cmd)
	ld.add_action(declare_urdf_model_path_cmd)
	ld.add_action(declare_use_robot_state_pub_cmd) 
	ld.add_action(declare_use_sim_time_cmd)
	ld.add_action(declare_use_simulator_cmd)
	ld.add_action(declare_world_cmd)
	ld.add_action(start_gazebo_server_cmd)
	ld.add_action(start_gazebo_client_cmd)

	
	ld.add_action(spawn_ball_cmd)
	ld.add_action(spawn_pin1_cmd)
	ld.add_action(spawn_pin2_cmd)
	ld.add_action(spawn_pin3_cmd)
	ld.add_action(spawn_pin4_cmd)
	ld.add_action(spawn_pin5_cmd)
	ld.add_action(spawn_pin6_cmd)
	ld.add_action(start_robot_state_publisher_cmd)

	return ld
