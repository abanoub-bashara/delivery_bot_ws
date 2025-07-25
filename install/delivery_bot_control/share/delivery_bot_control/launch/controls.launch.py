#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch-time arguments for your wheel geometry
    declare_wheel_radius = DeclareLaunchArgument(
        'wheel_radius', default_value='0.1',
        description='Radius of each omni wheel (meters)')
    declare_wheel_distance = DeclareLaunchArgument(
        'wheel_distance', default_value='0.5',
        description='Distance from robot center to each wheel (meters)')

    wheel_radius   = LaunchConfiguration('wheel_radius')
    wheel_distance = LaunchConfiguration('wheel_distance')

    return LaunchDescription([
        # make the arguments available
        declare_wheel_radius,
        declare_wheel_distance,

        # 1) Hardware bridge (on real Pi; skipped in Gazebo sim)
        Node(
            package='delivery_bot_control',
            executable='hardware_bridge_node',
            name='hardware_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),

        # 2) Omni-kinematics (cmd_vel → wheel traj)
        Node(
            package='delivery_bot_control',
            executable='omni_kinematics_node.py',
            name='omni_kinematics',
            output='screen',
            parameters=[{
                'wheel_radius':   wheel_radius,
                'wheel_distance': wheel_distance,
                'use_sim_time':   False
            }]
        ),

        # 3) Odometry (wheel_states → /odom + TF)
        Node(
            package='delivery_bot_control',
            executable='odometry_node.py',
            name='odometry',
            output='screen',
            parameters=[{
                'wheel_radius':   wheel_radius,
                'wheel_distance': wheel_distance,
                'use_sim_time':   False
            }]
        ),
    ])
