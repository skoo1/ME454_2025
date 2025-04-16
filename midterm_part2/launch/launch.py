import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    path_pkgshare_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    path_pkgshare_this = FindPackageShare(package='midterm_part2').find('midterm_part2')

    path_world = os.path.join(path_pkgshare_this, 'worlds', 'empty_world.world')
    path_urdf_racket = os.path.join(path_pkgshare_this, 'rsc', 'urdf', 'tennis_racket.urdf')

    # Launch gazebo server and client

    ld_gazebo_server = IncludeLaunchDescription(
	PythonLaunchDescriptionSource([os.path.join("/opt/ros/humble/share/gazebo_ros", 'launch', 'gzserver.launch.py')]),
	    launch_arguments={'world': path_world,"verbose": "false", "pause": "true"}.items())
	
    ld_gazebo_client = IncludeLaunchDescription(
	    PythonLaunchDescriptionSource([os.path.join("/opt/ros/humble/share/gazebo_ros", 'launch', 'gzclient.launch.py')]))

    # Spawn the URDF by topic

    ld_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', path_urdf_racket])}])

    ld_gazebo_spawn_racket = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tennis_racket',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '1.0'
        ],
        output='screen'
    )

    # Run ROS2 subscriber and printer nodes

    ld_robot_state_forwarder = Node(
        package='midterm_part2',
        executable='state_forwarder_20250000', # your student id here
        output='screen'
    )

    ld_robot_state_printer = Node(
        package='midterm_part2',
        executable='state_printer',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(ld_gazebo_server)
    ld.add_action(ld_gazebo_client)
    ld.add_action(ld_robot_state_publisher)
    ld.add_action(ld_gazebo_spawn_racket)
    ld.add_action(ld_robot_state_forwarder)
    ld.add_action(ld_robot_state_printer)

    return ld
