cmake_minimum_required(VERSION 3.8)
project(path_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(path_pub src/path_pub.cpp)

ament_target_dependencies(path_pub
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  ament_index_cpp
)

target_link_libraries(path_pub yaml-cpp)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  path_pub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
