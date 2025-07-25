import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('delivery_bot_nav')
    default_map_yaml = os.path.join(pkg_share, 'maps', 'map.yaml')
    default_amcl_yaml = os.path.join(pkg_share, 'config', 'amcl_params.yaml')

    # launch arguments
    map_arg = DeclareLaunchArgument(
        'map', default_value=default_map_yaml,
        description='Full path to map file')
    amcl_arg = DeclareLaunchArgument(
        'params_file', default_value=default_amcl_yaml,
        description='AMCL parameter file')
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time')

    # map_server node
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': LaunchConfiguration('map')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # amcl node
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    lifecycle_mgr = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_localization',
    output='screen',
    parameters=[{
      'use_sim_time': True,
      'autostart':    True,
      'node_names':   ['map_server', 'amcl']
    }]
)

    return LaunchDescription([
        map_server,
        amcl,
        lifecycle_mgr
    ])


