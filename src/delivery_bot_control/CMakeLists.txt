cmake_minimum_required(VERSION 3.8)
project(delivery_bot_control)

find_package(ament_cmake       REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(rclcpp_components  REQUIRED)

find_package(sensor_msgs        REQUIRED)
find_package(trajectory_msgs    REQUIRED)
find_package(geometry_msgs      REQUIRED)
find_package(nav_msgs           REQUIRED)
find_package(tf2_ros            REQUIRED)
find_package(control_msgs       REQUIRED)

find_package(pluginlib         REQUIRED)
find_package(hardware_interface REQUIRED)

# bring in the ros2_control core & controllers
find_package(ros2_control       REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_controllers   REQUIRED)
find_package(gz_ros2_control REQUIRED)  # CHANGED: gazebo_ros2_control -> gz_ros2_control

# # 1) C++ hardware‐bridge node
# add_executable(hardware_bridge_node src/hardware_bridge_node.cpp)
# ament_target_dependencies(hardware_bridge_node
#   rclcpp
#   sensor_msgs
#   trajectory_msgs
#   geometry_msgs
#   control_msgs
#   hardware_interface
#   pluginlib
# )
# install(
#   TARGETS hardware_bridge_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# 2) Python kinematics + odometry nodes
install(
  PROGRAMS
    src/omni_kinematics_node.py
    src/odometry_node.py
    src/omni_velocity_smoother.py
  DESTINATION lib/${PROJECT_NAME}
)

# 3) Config & launch
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# tell pluginlib where to find your hardware_sim XML
pluginlib_export_plugin_description_file(
  hardware_interface
  DefaultRobotHWSim.xml
)

ament_package()