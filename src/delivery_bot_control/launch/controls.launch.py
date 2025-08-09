# delivery_bot_control/launch/control.launch.py
import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from xacro import process_file

def generate_launch_description():
    # Paths
    description_pkg = get_package_share_directory('my_robot_description')
    xacro_path = os.path.join(description_pkg, 'urdf', 'robot_v3.xacro')
    robot_desc = process_file(xacro_path).toxml()

    control_pkg = get_package_share_directory('delivery_bot_control')
    config_path = os.path.join(control_pkg, 'config', 'omni_drive_controller.yaml')

    return LaunchDescription([
        # Start ros2_control_node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_desc},
                config_path,
                {'use_sim_time': True}
            ],
            output='screen'
        ),

        # Wait, then spawn joint state broadcaster
        TimerAction(period=3.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ]),

        # Wait, then spawn your omni controller
        TimerAction(period=5.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['omni_wheel_drive_controller', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ]),

        # Wait, then start your custom kinematics node
        TimerAction(period=7.0, actions=[
            Node(
                package='delivery_bot_control',
                executable='omni_kinematics_node.py',
                name='omni_kinematics',
                output='screen',
                parameters=[{
                    'wheel_radius': 0.024,
                    'robot_radius': 0.1155,
                    'use_sim_time': True,
                }]
            )
        ]),

        # Wait, then start your custom odometry node
        TimerAction(period=8.0, actions=[
            Node(
                package='delivery_bot_control',
                executable='odometry_node.py',
                name='odometry',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]),
    ])
