from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_description')
    urdf_path = os.path.join(pkg_path, 'urdf', 'robot_v2.urdf')
    
    # Read URDF file
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()
    
    return LaunchDescription([
        # Set Gazebo resource paths to help it find meshes
        # SetEnvironmentVariable(
        #     'GAZEBO_RESOURCE_PATH',
        #     os.path.join(pkg_path, '..')
        # ),
        
        # # Set Gazebo model path to include your package
        # SetEnvironmentVariable(
        #     'GAZEBO_MODEL_PATH',
        #     os.path.join(pkg_path, '..')
        # ),
        
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'verbose': 'true',  # Enable verbose output for debugging
                'gui': 'true'
            }.items()
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # Spawn robot entity
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'delivery_bot',
                '-file', urdf_path,
                '-x', '0.0',
                '-y', '0.0', 
                '-z', '0.5', 
                '-timeout', '60.0'  # Add timeout for spawn
            ],
            output='screen'
        )
    ])