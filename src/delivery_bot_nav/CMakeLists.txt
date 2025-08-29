cmake_minimum_required(VERSION 3.8)
project(delivery_bot_nav)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
  src/twist_to_stamped_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install your configs, launch files, and maps
install(
  DIRECTORY
    src
    config
    launch
    maps
    behaviour_trees
    rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()