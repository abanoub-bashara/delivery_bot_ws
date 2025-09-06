cmake_minimum_required(VERSION 3.8)
project(delivery_bot_description)

find_package(robot_localization REQUIRED)
find_package(ament_cmake REQUIRED)


install(DIRECTORY
  models
  urdf
  launch
  sdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()