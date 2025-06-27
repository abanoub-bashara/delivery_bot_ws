from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the Gazebo launch file location
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    robot_desc_pkg = get_package_share_directory('robot_description')

    urdf_path = os.path.join(robot_desc_pkg, 'urdf', 'delivery_bot.urdf')

    return LaunchDescription([
        # Start Gazebo with ROS integration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
            )
        ),

        # Publish robot_state from the URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_path]
        ),

        # Spawn the robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', urdf_path, '-entity', 'delivery_bot'],
            output='screen'
        ),
    ])
