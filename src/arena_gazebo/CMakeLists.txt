cmake_minimum_required(VERSION 3.8)
project(arena_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/traffic_light_controller.py DESTINATION lib/${PROJECT_NAME})

ament_package()
