#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    robot_desc_pkg = FindPackageShare("my_robot_description")
    ctrl_pkg       = FindPackageShare("delivery_bot_control")

    # same xacro
    robot_description = ParameterValue(
      Command([
        "xacro ",
        PathJoinSubstitution([robot_desc_pkg, "urdf", "robot_v3.xacro"])
      ]),
      value_type=str,
    )

    # 1) controller_manager
    controller_manager = Node(
      package="controller_manager",
      executable="ros2_control_node",
      parameters=[
        {"robot_description": robot_description},
        PathJoinSubstitution([ctrl_pkg, "config", "ros2_control.yaml"]),
      ],
      output="screen",
    )

    # 2) spawners, _delayed_ so /controller_manager service is ready
    spawn_jsb = TimerAction(
      period=3.0,
      actions=[ Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
          "joint_state_broadcaster",
          "--controller-manager", "/controller_manager"
        ],
        output="screen",
      ) ]
    )

    spawn_omni = TimerAction(
      period=5.0,
      actions=[ Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
          # **must** match your YAML name exactly:
          "omni_wheel_drive_controller",
          "--controller-manager", "/controller_manager"
        ],
        output="screen",
      ) ]
    )

    return LaunchDescription([
      controller_manager,
      spawn_jsb,
      spawn_omni,
    ])
