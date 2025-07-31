#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    desc_pkg   = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(desc_pkg, 'urdf', 'robot_v3.xacro')

    # 1) robot_description only
    robot_description = {
      'robot_description': ParameterValue(
           Command(['xacro ', xacro_file]), value_type=str
      )
    }

    # 2) publish TFs
    rsp_node = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      parameters=[robot_description, {'use_sim_time': True}],
      output='screen',
    )

    # 3) start Gazebo
    gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(
          get_package_share_directory('gazebo_ros'),
          'launch', 'gazebo.launch.py'
        )
      ),
      launch_arguments={
        'verbose':'true','gui':'true','pause':'false','use_sim_time':'true'
      }.items(),
    )

    # 4) spawn the robot
    spawn_entity = Node(
      package='gazebo_ros', executable='spawn_entity.py',
      arguments=[
        '-topic','robot_description','-entity','delivery_bot',
        '-timeout','60.0','-x','0.0','-y','0.0','-z','0.5'
      ],
      output='screen',
    )

    return LaunchDescription([
      rsp_node,
      gazebo,
      spawn_entity,
    ])
