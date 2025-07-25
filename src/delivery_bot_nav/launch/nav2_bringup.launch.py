import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_nav = get_package_share_directory('nav2_bringup')
    pkg_share = get_package_share_directory('delivery_bot_nav')

    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_map    = os.path.join(pkg_share, 'maps', 'map.yaml')

    # launch arguments
    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Full path to Nav2 parameters file')
    map_arg = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to map file')
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time')

    # include the standard bringup launch from Nav2
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map':           LaunchConfiguration('map'),
            'params_file':   LaunchConfiguration('params_file'),
            'use_sim_time':  LaunchConfiguration('use_sim_time')
        }.items()
    )

    return LaunchDescription([
        params_arg, map_arg, use_sim_time,
        bringup
    ])
