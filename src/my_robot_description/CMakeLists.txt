cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

find_package(robot_localization REQUIRED)
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  models
  urdf
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()