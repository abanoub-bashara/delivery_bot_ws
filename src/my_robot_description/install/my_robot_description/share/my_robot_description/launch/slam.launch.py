#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Minimal test launch for slam_toolbox to isolate autostart issue
    """
    
    # Test 1: Absolute minimal async slam
    slam_minimal = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
    )
    
    return LaunchDescription([
        slam_minimal,
    ])