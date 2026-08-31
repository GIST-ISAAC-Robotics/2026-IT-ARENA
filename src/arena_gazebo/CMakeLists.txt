cmake_minimum_required(VERSION 3.8)
project(arena_gazebo)

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

add_library(arena-single-motor-system SHARED src/single_motor_drive.cpp)
target_compile_features(arena-single-motor-system PRIVATE cxx_std_17)
target_include_directories(arena-single-motor-system PRIVATE include)
target_link_libraries(arena-single-motor-system PRIVATE gz-sim8::gz-sim8 gz-plugin2::gz-plugin2)
target_compile_options(arena-single-motor-system PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS arena-single-motor-system LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_drivetrain test/test_drivetrain.cpp)
  target_compile_features(test_drivetrain PRIVATE cxx_std_17)
  target_include_directories(test_drivetrain PRIVATE include)
  add_test(NAME single_motor_equations COMMAND test_drivetrain)
endif()

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/traffic_light_controller.py DESTINATION lib/${PROJECT_NAME})

ament_package()
