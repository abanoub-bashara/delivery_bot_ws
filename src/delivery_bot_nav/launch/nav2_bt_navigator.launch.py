import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('delivery_bot_nav')

    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_bt     = os.path.join(pkg_share, 'config', 'behaviour_trees',
                                  'navigate_w_replanning_and_recovery.xml')

    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Nav2 parameter file')
    bt_arg = DeclareLaunchArgument(
        'bt_file', default_value=default_bt,
        description='Behavior Tree XML file')
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time')

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[LaunchConfiguration('params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['--ros-args', '-p', 
                   'bt_xml_filename:=' + LaunchConfiguration('bt_file')]
    )

    return LaunchDescription([
        params_arg, bt_arg, use_sim_time,
        bt_navigator
    ])
