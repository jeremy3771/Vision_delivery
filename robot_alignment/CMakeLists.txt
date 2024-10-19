cmake_minimum_required(VERSION 3.8)
project(robot_alignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(alignment src/alignment.cpp)
ament_target_dependencies(alignment rclcpp geometry_msgs sensor_msgs)

install(TARGETS
  alignment
  DESTINATION lib/${PROJECT_NAME})

ament_package()
