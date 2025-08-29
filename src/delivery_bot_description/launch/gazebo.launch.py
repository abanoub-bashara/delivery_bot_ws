#gazebo.launch.py
import os
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, 
                            TimerAction, 
                            IncludeLaunchDescription
                            )

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from xacro import process_file
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Paths ---
    description_pkg = get_package_share_directory('delivery_bot_description')
    control_pkg = get_package_share_directory('delivery_bot_control')
    nav_pkg = get_package_share_directory('delivery_bot_nav')

    #load urdf description 
    xacro_path = os.path.join(description_pkg, 'urdf', 'robot_v3.xacro')
    robot_desc = process_file(xacro_path).toxml()

    #load config files for nodes
    teleop_config_file = os.path.join(control_pkg, 'config', 'teleop_params.yaml')
    joy_config_file = os.path.join(control_pkg, 'config', 'joy_params.yaml')
    ekf_config_file = os.path.join(control_pkg, 'config', 'ekf_params.yaml')

    # Built‑in empty world for Ignition Gazebo 6 (Fortress)
    world_path = '/home/abanoub/delivery_bot_ws/src/my_worlds/worlds/empty_with_lidar.sdf'

    # Gazebo resource path
    gazebo_model_path = os.path.join(description_pkg, 'models')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gazebo_model_path += ':' + os.environ['GZ_SIM_RESOURCE_PATH']



    # 1) Start Gazebo (server+GUI) - CHANGED: Use 'gz sim' instead of 'ign gazebo'
    gz_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_path, '-r', '-v', '1'],  # Changed from 'ign gazebo'
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

    # 3) Bridge essentials - CHANGED: Updated bridge syntax
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',  # Fixed syntax
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',  # Fixed syntax
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
            '-name', 'delivery_bot', #v2 or no v
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
                    'omni_drive_controller',  # Changed from 'omni_wheel_drive_controller'
                    '--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '60.0',
                    '--switch-timeout', '60.0',
                ],
            )
        ]
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_odom',
        output='screen',
        parameters=[ekf_config_file],
        # remappings=[
        #     ('/odometry/filtered', '/odom'),  # Remap EKF output from /odometry/filtered to /odom
        # ]
    )


    # slam_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(get_package_share_directory('delivery_bot_slam'), 
    #                     'launch', 'slam.launch.py')
    #     ])
    # )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[joy_config_file],   # joystick params
        output='screen'
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',       # <-- this is the correct executable
        name='teleop_twist_joy_node',
        parameters=[teleop_config_file,
                    {'publish_stamped_twist': True}], # teleop params
        remappings=[('/cmd_vel', '/omni_drive_controller/cmd_vel')],
        output='screen'
    )


    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}],
    # )


    return LaunchDescription([
        gz_gazebo,
        TimerAction(
            period=5.0,
            actions=[
                state_pub,
                bridge,
                spawn,
                wait_for_controllers,
                spawn_drive_controller,
            ]
        ),
        TimerAction(
            period=5.0,
            actions=[
                ekf,
                joy_node,
                teleop_node,
                # slam_launch,
                # rviz
            ]
        )
    ])