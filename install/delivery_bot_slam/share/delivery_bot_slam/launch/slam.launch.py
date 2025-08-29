import os
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    slam_pkg = get_package_share_directory('delivery_bot_slam')
    slam_config_file = os.path.join(slam_pkg, 'config', 'slam_params.yaml')

    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='async_slam_toolbox_node',     # <-- must match the name below in node_names
        namespace='',            # <-- REQUIRED in Jazzy even if empty
        output='screen',
        parameters=[slam_config_file]
    )

    # 2) Lifecycle manager that will auto-configure/activate slam_toolbox
    manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'autostart': True,            # drive transitions automatically
            'node_names': ['async_slam_toolbox_node'],  # must match the LifecycleNode name
            'use_sim_time': True,
            'bond_timeout': 15.0,  # Increase timeout
            'attempt_respawn_reconnection': True,
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        slam,
        manager
    ])