#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_cv = get_package_share_directory('cv')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world_file = os.path.join(pkg_cv, 'worlds', 'MyWorld.world')

    # Gazebo server and client
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot state publisher
    urdf_file = os.path.join(pkg_turtlebot3_description, 'urdf', 'turtlebot3_waffle_pi.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}, {'use_sim_time': use_sim_time}]
    )

    # Spawn TurtleBot
    spawn_turtlebot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'turtlebot3_waffle_pi',
                   '-file', os.path.join(pkg_cv, 'models', 'turtlebot3_waffle_pi', 'model.sdf'),
                   '-x', x_pose, '-y', y_pose, '-z', '0.01'],
        output='screen'
    )

    # --- CORRECTED: Spawn traffic signs with unique variables and correct model paths ---
    spawn_traffic_right = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'traffic_right',
                   '-file', os.path.join(pkg_cv, 'models', 'traffic_right', 'model.sdf'),
                   '-x', '3.0', '-y', '0.0', '-z', '0.0', '-Y', '1.5708'],
        output='screen'
    )
    
    spawn_traffic_left = Node( 
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'traffic_left',
                   '-file', os.path.join(pkg_cv, 'models', 'traffic_left', 'model.sdf'), 
                   '-x', '2.3', '-y', '-5.0', '-z', '0.0' ],
        output='screen'
    )

    spawn_traffic_stop = Node( # <-- UNIQUE NAME
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'traffic_stop',
                   '-file', os.path.join(pkg_cv, 'models', 'traffic_stop', 'model.sdf'), 
                   '-x', '5.8', '-y', '-4.3', '-z', '0.0', '-Y', '1.5708'],
        output='screen'
    )

    # Sign detector node
    sign_detector_node = Node(
        package='sign_detector',
        executable='detector_node',
        name='sign_detector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Controller node
    controller_node = Node(
        package='cv',
        executable='controller',
        name='controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- CORRECTED: Add all three spawn nodes to the return list ---
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        gzserver_launch,
        gzclient_launch,
        robot_state_publisher,
        spawn_turtlebot,
        spawn_traffic_right,
        spawn_traffic_left, # <-- ADDED
        spawn_traffic_stop, # <-- ADDED
        sign_detector_node,
        controller_node
    ])