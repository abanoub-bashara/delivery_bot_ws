# delivery_bot_description/launch/gazebo_with_control.launch.py
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

    # Gazebo resource path
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

    # 4) Spawn robot
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

    # 5) Controller spawners
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-type', 'joint_state_broadcaster/JointStateBroadcaster',
            '--controller-manager-timeout', '30.0',
            '--switch-timeout', '30.0'
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
            '--controller-manager-timeout', '35.0',
            '--switch-timeout', '35.0'
        ],
    )

    # 6) YOUR CUSTOM NODES
    # Kinematics node (converts cmd_vel to wheel commands)
    kinematics_node = Node(
        package='delivery_bot_control',  # Adjust package name if different
        executable='omni_kinematics_node.py',
        output='screen',
        parameters=[{
            'wheel_radius': 0.024,
            'robot_radius': 0.1155,
            'wheel_angles_deg': [0.0, 120.0, 240.0],
            'cmd_rate': 50.0,
            'cmd_timeout': 0.5,
            'use_sim_time': True
        }]
    )

    # Odometry node (publishes odometry from joint states)
    odometry_node = Node(
        package='delivery_bot_control',  # Adjust package name if different
        executable='odometry_node.py',
        output='screen',
        parameters=[{
            'wheel_radius': 0.024,
            'robot_radius': 0.1155,
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        ign_gazebo,
        state_pub,
        bridge,
        spawn,
        
        # Start controllers after robot spawns
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn,
                on_exit=[
                    TimerAction(period=3.0, actions=[jsb_spawner]),
                    TimerAction(period=5.0, actions=[drive_spawner]),
                    # Start your custom nodes after controllers are ready
                    TimerAction(period=7.0, actions=[kinematics_node]),
                    TimerAction(period=8.0, actions=[odometry_node]),
                ]
            )
        ),
    ])