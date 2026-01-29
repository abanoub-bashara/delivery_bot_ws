#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directory
    nav_pkg = get_package_share_directory('delivery_bot_nav')
    
    # Launch arguments
    locations_file = LaunchConfiguration('locations_file')
    
    # Declare launch arguments
    declare_locations_file_cmd = DeclareLaunchArgument(
        'locations_file',
        default_value=os.path.join(nav_pkg, 'config', 'saved_locations.yaml'),
        description='Full path to saved locations YAML file'
    )
    
    # Location Manager Node
    location_manager_node = Node(
        package='delivery_bot_nav',
        executable='location_manager',   # no .py
        name='location_manager',
        output='screen',
        parameters=[{
            'locations_file': locations_file,
            'use_sim_time': True
        }]
    )

    # Command Interface Node (optional - can run separately)
    command_interface_node = Node(
        package='delivery_bot_nav',
        executable='command_interface',  # no .py
        name='command_interface',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        # Launch Arguments
        declare_locations_file_cmd,
        
        # Nodes
        location_manager_node,
        # Uncomment the next line if you want command interface to start automatically
        # command_interface_node,
    ])
