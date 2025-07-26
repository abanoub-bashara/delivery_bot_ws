#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from xacro import process_file

def generate_launch_description():
    # Locate robot_description (Xacro → URDF) - NO MAPPINGS
    pkg_path = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot_v2.xacro')
    doc = process_file(xacro_file)  # Remove the mappings parameter
    robot_description = doc.toxml()

    # Controller config file (YAML)
    control_pkg_path = get_package_share_directory('delivery_bot_control')
    controller_yaml = os.path.join(control_pkg_path, 'config', 'omni_controller.yaml')

    # 1) Launch Gazebo (empty world)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'verbose': 'true',
            'gui': 'true',
            'pause': 'false',
            'use_sim_time': 'true',
        }.items()
    )

    # 2) Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ]
    )

    # 3) Spawn robot into Gazebo
    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', 'delivery_bot',
            '-topic', 'robot_description',
            '-timeout', '60.0',
        ]
    )

    # 4) ros2_control Node (after spawn)
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
            controller_yaml,
        ],
        output='screen'
    )

    # 5) Spawn joint_state_broadcaster
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # 6) Spawn omni_controller
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['omni_controller'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp_node,
        spawn_node,
        control_node,
        joint_state_spawner,
        controller_spawner
    ])