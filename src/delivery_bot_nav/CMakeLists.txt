cmake_minimum_required(VERSION 3.8)
project(delivery_bot_nav)

find_package(ament_cmake REQUIRED)

# Install your configs, launch files, and maps
install(
  DIRECTORY
    config
    launch
    maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()