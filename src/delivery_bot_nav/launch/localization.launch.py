import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directories
    nav_pkg = get_package_share_directory('delivery_bot_nav')
    
    # Launch arguments
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    
    # Declare launch arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav_pkg, 'maps', 'my_map2.yaml'),
        description='Full path to map yaml file to load'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    # Config files
    nav2_params_file = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    
    # Map server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_file,
            'use_sim_time': use_sim_time
        }]
    )
    
    # AMCL localization - FIXED: Added proper remappings
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('scan', '/scan'),  # Ensure scan topic is correct
            ('odom', '/odometry/filtered')  # Use EKF filtered odometry
        ]
    )

        # --- Localization lifecycle manager (what the RViz panel expects) ---
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'bond_timeout': 0.0,     # optional but handy in sim
            'node_names': ['map_server', 'amcl']
        }]
    )


    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        # Launch Arguments
        declare_map_yaml_cmd,
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        
        # Nodes
        map_server_node,
        amcl_node,
        lifecycle_manager_localization,
    ])