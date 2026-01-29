#!/usr/bin/env python3
import os
from launch import LaunchDescription 
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Launch arguments ---
    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map')
    locations_file = LaunchConfiguration('locations_file')
    start_cli = LaunchConfiguration('start_cli')

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='navigation',
        description='Launch mode: "slam" for mapping, "navigation" for autonomous navigation'
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            get_package_share_directory('delivery_bot_nav'),
            'maps', 'my_map2.yaml'
        ),
        description='Full path to map yaml file (used in navigation mode)'
    )

    declare_locations_file_cmd = DeclareLaunchArgument(
        'locations_file',
        default_value=os.path.join(
            get_package_share_directory('delivery_bot_nav'),
            'config', 'saved_locations.yaml'
        ),
        description='Full path to saved locations YAML'
    )

    declare_start_cli_cmd = DeclareLaunchArgument(
        'start_cli',
        default_value='false',
        description='Start the text-based command interface (go to/save/list/stop)'
    )

    # --- Package directories ---
    desc_pkg = get_package_share_directory('delivery_bot_description')
    nav_pkg = get_package_share_directory('delivery_bot_nav')
    slam_pkg = get_package_share_directory('delivery_bot_slam')

    # --- RViz configs ---
    rviz_nav_config = os.path.join(nav_pkg, 'rviz', 'nav.rviz')
    rviz_slam_config = os.path.join(nav_pkg, 'rviz', 'slam.rviz')

    # 1) Gazebo + robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(desc_pkg, 'launch', 'gazebo.launch.py')
        ])
    )

    # 2) SLAM mode (mapping)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(slam_pkg, 'launch', 'slam.launch.py')
        ]),
        condition=IfCondition(PythonExpression(['"', mode, '" == "slam"']))
    )

    # 3) Navigation mode (Nav2 bringup)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav_pkg, 'launch', 'navigation.launch.py')
        ]),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
            'autostart': 'true'
        }.items(),
        condition=IfCondition(PythonExpression(['"', mode, '" == "navigation"']))
    )

    # 4) Location Manager (services: save/get/list + YAML persistence)
    location_manager_node = Node(
        package='delivery_bot_nav',
        executable='location_manager.py',   # console_scripts entry point
        name='location_manager',
        output='screen',
        parameters=[{
            'locations_file': locations_file,
            'use_sim_time': True
        }],
        # Only really useful in navigation mode, but harmless in slam; start in both if you prefer
        condition=IfCondition(PythonExpression(['"', mode, '" == "navigation"']))
    )

    # 5) Command Interface (text "voice commands"; optional)
    # Starts a few seconds later so Nav2 + services are up.
    start_command_interface = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='delivery_bot_nav',
                executable='command_interface.py',  # console_scripts entry point
                name='command_interface',
                output='screen',
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(
                    PythonExpression(['(', '"', mode, '" == "navigation"', ') and (', '"', start_cli, '" == "true"', ')'])
                )
            )
        ]
    )

    # RViz (navigation)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_nav_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(PythonExpression(['"', mode, '" == "navigation"']))
    )

    # RViz (slam)
    rviz_slam_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_slam_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(PythonExpression(['"', mode, '" == "slam"']))
    )

    return LaunchDescription([
        declare_mode_cmd,
        declare_map_cmd,
        declare_locations_file_cmd,
        declare_start_cli_cmd,

        gazebo_launch,
        slam_launch,
        navigation_launch,

        # Saved-locations services (navigation mode)
        location_manager_node,

        # Optional text "voice" CLI
        start_command_interface,

        # Delay RViz a bit (your existing pattern)
        TimerAction(
            period=10.0,
            actions=[rviz_node, rviz_slam_node]
        ),
    ])
