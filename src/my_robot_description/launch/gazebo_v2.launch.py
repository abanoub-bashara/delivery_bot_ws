# delivery_bot_description/launch/gazebo_v2.launch.py
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from xacro import process_file
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- Paths ---
    description_pkg = get_package_share_directory('my_robot_description')
    xacro_path = os.path.join(description_pkg, 'urdf', 'robot_v3.xacro')
    robot_desc = process_file(xacro_path).toxml()

    # Built‑in empty world for Ignition Gazebo 6 (Fortress)
    world_path = '/home/abanoub/delivery_bot_ws/src/my_worlds/worlds/empty_with_lidar.sdf'

    # Gazebo resource path (for model:// lookups). Your URDF uses file://, so this is just a helper.
    gazebo_model_path = os.path.join(description_pkg, 'models')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gazebo_model_path += ':' + os.environ['GZ_SIM_RESOURCE_PATH']

    # --- Nodes ---

    # 1) Start Gazebo (server+GUI)
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_path, '--render-engine', 'ogre2', '-r', '-v', '1'],
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': gazebo_model_path},
    )

    # 2) Publish /robot_description
    state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }],
    )

    # 3) Bridge essentials (clock + lidar topics)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 4) Spawn robot from /robot_description into the "empty" world
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-name', 'delivery_bot',
            '-topic', 'robot_description',
            '-world', 'empty_with_lidar',
            '-x', '0', '-y', '0', '-z', '1',
        ],
    )

    # 5) Controller spawners — wait for controller_manager to be up
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-type', 'joint_state_broadcaster/JointStateBroadcaster',
            '--controller-manager-timeout', '20.0',
            '--switch-timeout', '10.0'
        ],
    )

    drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'omni_wheel_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-type', 'velocity_controllers/JointGroupVelocityController',
            '--param-file', '/home/abanoub/delivery_bot_ws/src/delivery_bot_control/config/omni_drive_controller.params.yaml',
            '--controller-manager-timeout', '30.0',
            '--switch-timeout', '30.0'
        ],
    )

    return LaunchDescription([
        ign_gazebo,
        state_pub,
        bridge,
        spawn,
        # Use TimerAction instead of OnProcessExit + sleep for better sequencing
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn,
                on_exit=[
                    TimerAction(
                        period=3.0,  # Wait 3 seconds after spawn completes
                        actions=[jsb_spawner]
                    ),
                    TimerAction(
                        period=5.0,  # Wait 5 seconds after spawn completes
                        actions=[drive_spawner]
                    ),
                ]
            )
        ),
    ])