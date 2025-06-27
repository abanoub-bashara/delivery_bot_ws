from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    robot_desc_pkg = get_package_share_directory('robot_description')

    xacro_path = os.path.join(robot_desc_pkg, 'urdf', 'delivery_bot.xacro')
    rviz_config_path = os.path.join(robot_desc_pkg, 'rviz', 'delivery_bot.rviz')
    controller_yaml_path = os.path.join(robot_desc_pkg, 'config', 'controller_config.yaml')

    robot_description = {'robot_description': Command(['xacro ', xacro_path])}

    return LaunchDescription([
        # Start Gazebo (full GUI)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
            )
        ),

        # Start robot_state_publisher with processed xacro
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': True}]
        ),

        # Spawn controllers (these wait for controller_manager to come online)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager-timeout', '50'],
            parameters=[controller_yaml_path],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '--controller-manager-timeout', '50'],
            parameters=[controller_yaml_path],
            output='screen'
        ),

        # Spawn robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'delivery_bot', '-topic', 'robot_description'],
            output='screen'
        ),

        # Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])
