#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Define the world
    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_world.world'
    )

    # Simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot 1 position and entity name
    x_pose_1 = LaunchConfiguration('x_pose_1', default='-2.0')
    y_pose_1 = LaunchConfiguration('y_pose_1', default='-0.5')
    entity_name_1 = 'waffle_1'

    # Robot 2 position and entity name
    x_pose_2 = LaunchConfiguration('x_pose_2', default='2.0')
    y_pose_2 = LaunchConfiguration('y_pose_2', default='0.5')
    entity_name_2 = 'waffle_2'

    # Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Gazebo client (GUI)
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot 1: robot_state_publisher and spawn
    robot1_state_publisher_cmd = GroupAction([
        PushRosNamespace('tb3_1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose_1,
                'y_pose': y_pose_1,
                'entity_name': entity_name_1
            }.items()
        ),
    ])

    # Robot 2: robot_state_publisher and spawn
    robot2_state_publisher_cmd = GroupAction([
        PushRosNamespace('tb3_2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose_2,
                'y_pose': y_pose_2,
                'entity_name': entity_name_2
            }.items()
        ),
    ])

    # Create launch description and add actions
    ld = LaunchDescription()

    # Add the Gazebo server and client commands
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    # Add the commands to spawn both TurtleBot3 robots
    ld.add_action(robot1_state_publisher_cmd)
    ld.add_action(robot2_state_publisher_cmd)

    return ld
