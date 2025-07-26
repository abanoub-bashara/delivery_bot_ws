cmake_minimum_required(VERSION 3.8)
project(delivery_bot_control)

find_package(ament_cmake       REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(trajectory_msgs    REQUIRED)
find_package(geometry_msgs      REQUIRED)
find_package(nav_msgs           REQUIRED)
find_package(tf2_ros            REQUIRED)
find_package(PkgConfig          REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_control_test_assets REQUIRED)


# pigpio (for hardware_bridge)
# pkg_check_modules(PIGPIO REQUIRED pigpio)
# include_directories(${PIGPIO_INCLUDE_DIRS})
# link_directories(${PIGPIO_LIBRARY_DIRS})

# 1) C++ hardware bridge node
add_executable(hardware_bridge_node src/hardware_bridge_node.cpp)
ament_target_dependencies(hardware_bridge_node
  rclcpp
  sensor_msgs
  trajectory_msgs
)
target_link_libraries(hardware_bridge_node
  ${PIGPIO_LIBRARIES}
)
install(
  TARGETS hardware_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# 2) Python nodes
install(
  PROGRAMS
    src/omni_kinematics_node.py
    src/odometry_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 3) Config & launch files
install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)
pluginlib_export_plugin_description_file(hardware_interface DefaultRobotHWSim.xml)

ament_package()
