#gazebo.launch.py
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
    control_pkg = get_package_share_directory('delivery_bot_control')
    nav_pkg = get_package_share_directory('delivery_bot_nav')

    #load urdf description 
    xacro_path = os.path.join(description_pkg, 'urdf', 'robot_v3.xacro')
    robot_desc = process_file(xacro_path).toxml()

    #load config files for nodes
    teleop_config_file = os.path.join(control_pkg, 'config', 'teleop_joy.yaml')
    ekf_config_file = os.path.join(control_pkg, 'config', 'ekf.yaml')
    slam_config_file = os.path.join(nav_pkg, 'config', 'slam.params.yaml')

    # Built‑in empty world for Ignition Gazebo 6 (Fortress)
    world_path = '/home/abanoub/delivery_bot_ws/src/my_worlds/worlds/empty_with_lidar.sdf'

    # Gazebo resource path
    gazebo_model_path = os.path.join(description_pkg, 'models')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gazebo_model_path += ':' + os.environ['GZ_SIM_RESOURCE_PATH']

    # --- Nodes ---

    # 1) Start Gazebo (server+GUI)
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_path, '-r', '-v', '1'], #'-s' means headless (no gui)
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
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-name', 'delivery_bot',
            '-param', 'robot_description',
            '-world', 'empty_with_lidar',
            '-x', '0', '-y', '0', '-z', '0.5',
        ],
        parameters=[{'robot_description': robot_desc}],
    )
    

    # Wait a bit before spawning controllers to ensure gz_ros2_control is ready
    wait_for_controllers = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager', executable='spawner', output='screen',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '60.0',
                    '--switch-timeout', '60.0',
                ],
            )
        ]
    )

    # Spawn drive controller after JSB
    spawn_drive_controller = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='controller_manager', executable='spawner', output='screen',
                arguments=[
                    'omni_wheel_drive_controller',
                    '--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '60.0',
                    '--switch-timeout', '60.0',
                ],
            )
        ]
    )
    
    # wheel_test_node = TimerAction(
    #     period=20.0,  # Start after controllers are fully initialized
    #     actions=[
    #         Node(
    #             package='delivery_bot_control',
    #             executable='wheel_direction_test',  # Make sure this matches your executable name
    #             output='screen',
    #             parameters=[{
    #                 'wheel_radius': 0.024,
    #                 'robot_radius': 0.1155,
    #                 'wheel_angles_deg': [0.0, 120.0, 240.0],
    #                 'use_sim_time': True,
    #             }]
    #         )
    #     ]
    # )

    # 6) YOUR CUSTOM NODES
    kinematics_node = Node(
        package='delivery_bot_control',
        executable='omni_kinematics_node.py',
        output='screen',
        parameters=[{
            'wheel_radius': 0.024,
            'robot_radius': 0.1155,
            'wheel_angles_deg': [150.0, 30.0, 270.0],
            'cmd_rate': 50.0,
            'cmd_timeout': 0.5,
            'use_sim_time': True,
            'max_wheel_accel': 20.0,
            'max_wheel_speed': 80.0,
            'max_wheel_jerk': 0.0,
            # ADD THESE MISSING PARAMETERS:
            'ramp_mode': 'body_coordinated',
            'wheel_joint_names': ['wheel1_joint', 'wheel2_joint', 'wheel3_joint'],
            'sign_correction': [1.0, 1.0, 1.0]
        }]
    )

    odometry_node = Node(
        package='delivery_bot_control',
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
        parameters=[ekf_config_file],
        remappings=[
            ('odometry/filtered', '/odom'),  # Remap EKF output from /odometry/filtered to /odom
        ]
    )
    
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='async_slam_toolbox_node',
        output='screen',
        parameters=[{'use_sim_time': True}, 
                    slam_config_file],
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[teleop_config_file],
        output='screen'
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[teleop_config_file],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
        ],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        ign_gazebo,
        state_pub,
        bridge,
        spawn,
        wait_for_controllers,
        spawn_drive_controller,
        # wheel_test_node,
        kinematics_node,
        odometry_node,
        ekf,
        joy_node,
        teleop_node,
        slam,
        rviz,
    ])
