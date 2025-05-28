import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    path_to_cartpole = os.path.join(
        get_package_share_directory('cartpole'))

    xacro_file = os.path.join(path_to_cartpole,
                              'rsc',
                              'urdf',
                              'cartpoleeffort.urdf')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}
    
    # Include Gazebo client launch
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gzclient.launch.py'])
    )
    
    # Include Gazebo server launch with arguments
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gzserver.launch.py']),
        launch_arguments={
            'world': os.path.join(path_to_cartpole, 'worlds', 'cartpole.world'), 
            # using custom world, which has robot state publisher
            'pause': 'false'
        }.items()
    )

    # Node for the robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Node to spawn the entity in Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'cartpole'],
                        output='screen')

    node_load_PID = Node(
        package='cartpole',
        executable='pid_publisher_subscriber_node',
        name='pid_publisher_subscriber_node',
        output='screen'
    )

    # Load effort_controller1 controller
    load_effort_controller1 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'effort_controller1'],
        output='screen'
    )

    return LaunchDescription([
        gzclient,
        gzserver,
        node_robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_effort_controller1],
            )
        ),
        node_load_PID
    ])