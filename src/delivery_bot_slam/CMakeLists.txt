cmake_minimum_required(VERSION 3.8)
project(delivery_bot_slam)

find_package(robot_localization REQUIRED)
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()