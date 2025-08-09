# delivery_bot_description/launch/gz_min_spawn.launch.py
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from xacro import process_file

def generate_launch_description():
    description_pkg = get_package_share_directory('my_robot_description')
    xacro_path = os.path.join(description_pkg, 'urdf', 'robot_v3.xacro')
    robot_desc = process_file(xacro_path).toxml()

    world_path = '/home/abanoub/delivery_bot_ws/src/my_worlds/worlds/empty_with_lidar.sdf'
    model_path = os.path.join(description_pkg, 'models')

    ign = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '3', world_path],
        output='screen'
    )

    state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    spawn = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', 'delivery_bot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.5'
            # (omit -world; uses the only/active world)
        ],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', model_path),
        ign,
        state_pub,
        bridge,
        # give Gazebo a moment to load the world before spawning
        TimerAction(period=3.0, actions=[spawn]),
    ])
