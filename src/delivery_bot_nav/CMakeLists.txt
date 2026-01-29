cmake_minimum_required(VERSION 3.8)
project(delivery_bot_nav)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveLocation.srv"
  "srv/GetLocation.srv"
  "srv/ListLocations.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# Install Python executables
install(
  PROGRAMS
    src/twist_to_stamped_node.py
    src/location_manager.py
    src/command_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install your configs, launch files, and maps
install(
  DIRECTORY
    src
    config
    launch
    maps
    rviz
    srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()