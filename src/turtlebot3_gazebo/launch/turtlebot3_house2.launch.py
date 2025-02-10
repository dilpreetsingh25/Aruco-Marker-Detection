# <!-- <?xml version="1.0"?>
# <launch>
#     <!-- Define paths for URDF and RViz config -->
#     <let name="urdf_path" value="$(find-pkg-share turtlebot3_gazebo)/urdf/turtlebot3_waffle.urdf" />

#     <let name="rviz_config_path" value="$(find-pkg-share turtlebot3_gazebo)/rviz/tb3_gazebo.rviz" />

#     <!-- Robot State Publisher node -->
#     <node pkg="robot_state_publisher" exec="robot_state_publisher">
#         <param name="robot_description" value="$(command 'xacro $(var urdf_path)')" />
#     </node>

#     <include file="$(find-pkg-share gazebo_ros)/launch/gazebo.launch.py" >
#           <arg name="world" value="$(find-pkg-share my_robot_bringup)/worlds/test_world.world" />
#     </include>
     
#     <node pkg="gazebo_ros" exec="spawn_entity.py"
#           args="-topic robot_description -entity my_robot"/>
#     <!-- Joint State Publisher GUI node -->
#     <!-- <node pkg="joint_state_publisher_gui" exec="joint_state_publisher_gui"/> -->

#     <!-- RViz2 node -->
#     <node pkg="rviz2" exec="rviz2" output="screen" args="-d $(var rviz_config_path)" />
# </launch> -->

#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Paths to relevant package directories
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    karl_x_pose = LaunchConfiguration('karl_x_pose', default='-2.0')
    karl_y_pose = LaunchConfiguration('karl_y_pose', default='-0.5')

    hank_x_pose = LaunchConfiguration('hank_x_pose', default='2.0')
    hank_y_pose = LaunchConfiguration('hank_y_pose', default='0.5')

    # gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    # grobot_gazebo_pkg = get_package_share_directory('grobot_gazebo')
    # grobot_bringup_pkg = get_package_share_directory('grobot_bringup')

    # Arguments and configurations
    world_file = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                               'worlds', 'turtlebot3_house.world')

    turtlebot3_urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf', 'turtlebot3_waffle.urdf'
    )

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

    # Robot bringup commands for two robots, karl and hank
    # robot_karl_bringup_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
    #     ),
    #     # launch_arguments={'name': 'karl'}.items()
    #     launch_arguments={'use_sim_time': use_sim_time,'name':'karl'}.items()
    # )

    # robot_hank_bringup_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
    #     ),
    #     # launch_arguments={'name': 'hank'}.items(),
    #     launch_arguments={'use_sim_time': use_sim_time, 'name':'hank'}.items()
    # )
    
    # spawn_turtlebot_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
    #     ),
    #     launch_arguments={
    #         'x_pose': x_pose,
    #         'y_pose': y_pose
    #     }.items()
    # )
    # # Create the launch description and add the actions
    # ld = LaunchDescription()

    # # Add Gazebo commands
    # ld.add_action(gzserver_cmd)
    # ld.add_action(gzclient_cmd)

    # # Add robot bringup commands
    # ld.add_action(robot_karl_bringup_cmd)
    # ld.add_action(robot_hank_bringup_cmd)
    # ld.add_action(spawn_turtlebot_cmd)
    # return ld




    from launch_ros.actions import Node
    # Robot State Publisher for "karl"
    # robot_karl_state_publisher_cmd = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'robot_description': open(turtlebot3_urdf).read()
    #     }]
    # )

    # Robot State Publisher for "hank"
    robot_hank_state_publisher_cmd = Node(
        package=launch_file_dir,
        executable='robot_state_publisher.launch.py',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(turtlebot3_urdf).read()
        }]
    )

    # spawn_turtlebot_karl_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
    #     ),
    #     launch_arguments={
    #         'x_pose': karl_x_pose,
    #         'y_pose': karl_y_pose
    #     }.items()
    # )

    # spawn_turtlebot_hank_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
    #     ),
    #     launch_arguments={
    #         'x_pose': hank_x_pose,
    #         'y_pose': hank_y_pose
    #     }.items()
    # )

    # spawn_karl_cmd = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-topic', 'robot_description',
    #         '-entity', 'karl',
    #         '-x', karl_x_pose,
    #         '-y', karl_y_pose
    #     ],
    #     output='screen'
    # )

    # Spawn "hank"
    spawn_hank_cmd = Node(
        package=launch_file_dir,
        executable='spawn_turtlebot3.launch.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'hank',
            '-x', hank_x_pose,
            '-y', hank_y_pose
        ],
        output='screen'
    )

    # Create the launch description and add the actions
    ld = LaunchDescription()

    # Add Gazebo commands
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    # Add Robot State Publisher and spawn commands for "karl" and "hank"
    # ld.add_action(robot_karl_state_publisher_cmd)

    # ld.add_action(spawn_hank_cmd)
    
    ld.add_action(robot_hank_state_publisher_cmd)
 
    ld.add_action(spawn_hank_cmd)

    return ld

