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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node



def generate_launch_description():

	file_path = os.path.realpath(__file__)
	file_dir = os.path.dirname(file_path)
	(head_path, tail_path) = os.path.split(file_path)
	(head_path, tail_dir0) = os.path.split(head_path)
	(head_path, tail_dir1) = os.path.split(head_path)
	(head_path, tail_dir2) = os.path.split(head_path)
	if (tail_dir2 == 'src'): # symlink-install
		pkg_src_dir = os.path.join(file_dir, '..')
	else: # not symlink-install
		pkg_src_dir = os.path.join(file_dir, '..', '..', '..', '..', '..', 'src', 'pendulum_movement')
	
	world_file_name = 'pendulum.world'
	sdf_file_name = 'pendulum.sdf'
	world_path = os.path.join(pkg_src_dir, 'worlds', world_file_name)
	sdf_path = os.path.join(pkg_src_dir, 'rsc', sdf_file_name)

	return LaunchDescription([

		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(['/opt/ros/humble/share/gazebo_ros/launch/gzserver.launch.py']),
			launch_arguments={'world': world_path}.items()
		),

		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(['/opt/ros/humble/share/gazebo_ros/launch/gzclient.launch.py']),
		),

		Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', sdf_path, 
					   '-entity', 'pendulum',
					   '-z', '2.0',
					   '-P', '0.5'],
            output='screen'
        ),
	])

