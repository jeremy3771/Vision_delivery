cmake_minimum_required(VERSION 3.8)
project(regulated_pure_pursuit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)

add_executable(delivery_autonomous src/main.cpp src/regulated_pure_pursuit.cpp src/goal_checker.cpp)
ament_target_dependencies(delivery_autonomous
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  angles
  tf2_ros
  ament_index_cpp
)

target_link_libraries(delivery_autonomous yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  config param
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  delivery_autonomous
  DESTINATION lib/${PROJECT_NAME})

ament_package()
