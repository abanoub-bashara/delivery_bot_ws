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
    
    # Config file for teleop
    teleop_config_file = os.path.expanduser('~/delivery_bot_ws/src/delivery_bot_control/config/teleop_joy.yaml')

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
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 4) OLD SPAWNER
    # spawn = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     name='spawn_robot',
    #     output='screen',
    #     arguments=[
    #         '-name', 'delivery_bot',
    #         '-topic', 'robot_description',
    #         '-world', 'empty_with_lidar',
    #         '-x', '0', '-y', '0', '-z', '1',
    #     ],
    # )
    
    spawn = Node(
    package='ros_gz_sim',
    executable='create',
    name='spawn_robot',
    output='screen',
    arguments=[
        '-name', 'delivery_bot',
        '-param', 'robot_description',   # <-- read a ROS *parameter*
        '-world', 'empty_with_lidar',
        '-x', '0', '-y', '0', '-z', '1',
    ],
    parameters=[{'robot_description': robot_desc}],  # <-- give it the same XML
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
            'use_sim_time': True,
            'max_wheel_accel': 20.0,   # try 10–30
            'max_wheel_speed': 80.0,   # keep realistic
            'max_wheel_jerk': 0.0,     # set 50–200 to try jerk limiting, else 0
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
            'use_sim_time': True,
            'publish_tf': False, 
        }],
        remappings=[('/odom', '/wheel_odom')]  
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_odom',
        output='screen',
        parameters=['/home/abanoub/delivery_bot_ws/src/delivery_bot_control/config/ekf.yaml']
    )

    # Static TF for the scoped scan frame (matches lidarView pose in URDF)
    static_scan_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='scan_scoped_tf',
        # identity: parent = lidarView, child = delivery_bot/base_link/lidarView
        arguments=['0','0','0','0','0','0','lidarView','delivery_bot/base_link/lidarView'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 7) SLAM (slam_toolbox, async mode)
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            # frames & timing
            'use_sim_time': True,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'transform_publish_period': 0.05,   # 20 Hz map->odom

            # lidar limits (match your /lidar topic)
            'minimum_laser_range': 0.12,
            'maximum_laser_range': 6.0,

            # mapping behavior
            'mode': 'mapping',                  # (later you can switch to 'localization')
            'resolution': 0.05,                 # map resolution (m/cell)
            'scan_queue_size': 5,               # small queue; reduces lag/smear
            'throttle_scans': 1,                # use every scan

            # keyframe thresholds (helps avoid over-eager frames with little structure)
            'minimum_travel_distance': 0.05,    # 5 cm between keyframes
            'minimum_travel_heading': 0.05,     # ~3 degrees

            # keep a generous TF buffer for sim
            'tf_buffer_duration': 30.0,

            # leave odom publishing to your node; SLAM only publishes map->odom
            'provide_odom_frame': False
        }],
        arguments=['--ros-args', '-r', '/scan:=/lidar']
    )

    # 8) Joy node - reads PS4 controller input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[teleop_config_file],
        output='screen'
    )

    # 9) Teleop twist joy node - converts joy to twist messages
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[teleop_config_file],
        remappings=[
            ('cmd_vel', '/cmd_vel'),  # Adjust if your robot uses a different topic
        ],
        output='screen'
    )

    # 10) RViz2 (optional: point to a saved config if you have one)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        # If you have a config, add: arguments=['-d', '/path/to/your.rviz']
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
                    TimerAction(period=8.5, actions=[ekf]),
                    TimerAction(period=10.0, actions=[joy_node]),
                    TimerAction(period=11.0, actions=[teleop_node]),
                    TimerAction(period=12.0, actions=[static_scan_tf]),
                    TimerAction(period=16.0, actions=[slam]),
                    TimerAction(period=21.0, actions=[rviz]),
                ]
            )
        ),
    ])