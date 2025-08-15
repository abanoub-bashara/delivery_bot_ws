#gazebo_v2.launch.py
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from xacro import process_file
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Paths ---
    description_pkg = get_package_share_directory('my_robot_description')
    xacro_path = os.path.join(description_pkg, 'urdf', 'robot_v3.xacro')
    controller_config_file = os.path.expanduser('~/delivery_bot_ws/src/delivery_bot_control/config/controllers.yaml')
    robot_desc = process_file(xacro_path, mappings={'controller_config_file': controller_config_file}).toxml()
    teleop_config_file = os.path.expanduser('~/delivery_bot_ws/src/delivery_bot_control/config/teleop_joy.yaml')
    world_path = '/home/abanoub/delivery_bot_ws/src/my_worlds/worlds/empty_with_lidar.sdf'

    
    gazebo_model_path = os.path.join(description_pkg, 'models')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gazebo_model_path += ':' + os.environ['GZ_SIM_RESOURCE_PATH']


    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_path, '-r', '-v', '1'], 
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': gazebo_model_path},
    )

    # 2) Robot state publisher
    state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }],
    )


    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
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
            '-param', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.5',  # Lower spawn height
        ],
        parameters=[{'robot_description': robot_desc}],
    )

    # 5) LONGER delays for controller spawning due to slow RTF
    wait_for_joint_broadcaster = TimerAction(
        period=8.0,  # Increased delay
        actions=[
            Node(
                package='controller_manager', executable='spawner', output='screen',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '60.0',  # Much longer timeout
                    '--switch-timeout', '60.0',
                ],
            )
        ]
    )

    # 6) Even longer delay for drive controller
    spawn_drive_controller = TimerAction(
        period=15.0,  # Much longer delay
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
    kinematics_node = Node(
        package='delivery_bot_control',
        executable='omni_kinematics_node.py',
        output='screen',
        parameters=[{
            'wheel_radius': 0.024,
            'robot_radius': 0.1155,
            'wheel_angles_deg': [0.0, 120.0, 240.0],
            'cmd_rate': 50.0,
            'cmd_timeout': 0.5,
            'use_sim_time': True,
            'max_wheel_accel': 20.0,
            'max_wheel_speed': 80.0,
            'max_wheel_jerk': 0.0,
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
        parameters=['/home/abanoub/delivery_bot_ws/src/delivery_bot_control/config/ekf.yaml']
    )


    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'transform_publish_period': 0.05,
            'minimum_laser_range': 0.12,
            'maximum_laser_range': 6.0,
            'mode': 'mapping',
            'resolution': 0.05,
            'scan_queue_size': 5,
            'throttle_scans': 1,
            'minimum_travel_distance': 0.05,
            'minimum_travel_heading': 0.05,
            'tf_buffer_duration': 30.0,
            'provide_odom_frame': False
        }],
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
        wait_for_joint_broadcaster,
        spawn_drive_controller,
        kinematics_node,
        odometry_node,
        ekf,
        joy_node,
        teleop_node,
        slam,
        rviz,
    ])