#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Paths to relevant package directories
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Pose configuration for "karl" and "hank"
    karl_x_pose = LaunchConfiguration('karl_x_pose', default='-2.0')
    karl_y_pose = LaunchConfiguration('karl_y_pose', default='-0.5')

    hank_x_pose = LaunchConfiguration('hank_x_pose', default='2.0')
    hank_y_pose = LaunchConfiguration('hank_y_pose', default='0.5')

    # TurtleBot3 URDF path
    turtlebot3_urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf', 'turtlebot3_waffle.urdf'
    )

    # World file for Gazebo simulation
    world_file = os.path.join(turtlebot3_gazebo_dir, 'worlds', 'turtlebot3_house.world')

    # Gazebo server and client launch commands
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Joint State Publisher - this publishes the joint states for both robots
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # Robot State Publisher for "karl"
    robot_karl_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(turtlebot3_urdf).read()
        }]
    )

    # Robot State Publisher for "hank"
    robot_hank_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(turtlebot3_urdf).read()
        }]
    )

    # Spawn "karl" command
    spawn_karl_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'karl',
            '-x', karl_x_pose,
            '-y', karl_y_pose
        ],
        output='screen'
    )

    # Spawn "hank" command
    spawn_hank_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'hank',
            '-x', hank_x_pose,
            '-y', hank_y_pose
        ],
        output='screen'
    )

    # Create the launch description and add all the actions
    ld = LaunchDescription()

    # Add Gazebo server and client commands
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    # Add the joint state publisher for both robots
    ld.add_action(joint_state_publisher_cmd)

    # Add Robot State Publisher and spawn commands for both "karl" and "hank"
    ld.add_action(robot_karl_state_publisher_cmd)
    ld.add_action(spawn_karl_cmd)
    
    ld.add_action(robot_hank_state_publisher_cmd)
    ld.add_action(spawn_hank_cmd)

    return ld
