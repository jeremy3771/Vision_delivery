cmake_minimum_required(VERSION 3.8)
project(pointcloud_to_laserscan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pointcloud_to_laserscan src/pointcloud_to_laserscan.cpp)
ament_target_dependencies(pointcloud_to_laserscan rclcpp std_msgs sensor_msgs)

install(TARGETS
  pointcloud_to_laserscan
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  param
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
