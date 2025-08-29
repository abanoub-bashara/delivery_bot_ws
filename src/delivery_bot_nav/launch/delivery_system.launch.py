import os
from launch import LaunchDescription 
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map')
    
    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='navigation',
        description='Launch mode: "slam" for mapping, "navigation" for autonomous navigation'
    )
    
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            get_package_share_directory('delivery_bot_nav'), 
            'maps', 'my_map.yaml'
        ),
        description='Full path to map yaml file (used in navigation mode)'
    )
    
    # Package directories
    desc_pkg = get_package_share_directory('delivery_bot_description')
    nav_pkg = get_package_share_directory('delivery_bot_nav')
    slam_pkg = get_package_share_directory('delivery_bot_slam')
    
    # RViz config file
    rviz_nav_config = os.path.join(nav_pkg, 'rviz', 'nav.rviz')
    rviz_slam_config = os.path.join(nav_pkg, 'rviz', 'slam.rviz')

    # 1. Robot Simulation (Gazebo + Robot + Control + Teleop)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(desc_pkg, 'launch', 'gazebo.launch.py')
        ])
    )
    
    # 2. SLAM Launch (for mapping mode)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(slam_pkg, 'launch', 'slam.launch.py')
        ]),
        condition=IfCondition(
            PythonExpression(['"', mode, '" == "slam"'])
        )
    )
    
    # 3. Navigation Launch (for autonomous mode)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav_pkg, 'launch', 'navigation.launch.py')
        ]),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
            'autostart': 'true'
        }.items(),
        condition=IfCondition(
            PythonExpression(['"', mode, '" == "navigation"'])
        )
    )
    
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_nav_config],                 # <<<<<< key line
        parameters=[{'use_sim_time': True}],  # Remove the config file for now
        condition=IfCondition(
            PythonExpression(['"', mode, '" == "navigation"'])
        )
    )

    # For SLAM mode, use basic RViz config
    rviz_slam_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_slam_config],                 # <<<<<< key line
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(
            PythonExpression(['"', mode, '" == "slam"'])
        )
    )

    # Twist to TwistStamped bridge
    twist_bridge_node = Node(
        package='delivery_bot_nav',  # Use your existing package
        executable='twist_to_stamped_node.py',
        name='twist_to_stamped',
        output='screen'
    )
    
    return LaunchDescription([
        declare_mode_cmd,
        declare_map_cmd,
        
        # Always launch the robot simulation
        gazebo_launch,
        
        # Conditional launches based on mode
        slam_launch,
        navigation_launch,
        TimerAction(
            period=10.0,
            actions=[
                rviz_node,
                rviz_slam_node,
                twist_bridge_node
            ]
        )
    ])