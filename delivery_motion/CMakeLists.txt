cmake_minimum_required(VERSION 3.8)
project(delivery_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(run_zlac src/run_zlac.cpp src/zlac8015d.cpp)
ament_target_dependencies(run_zlac rclcpp geometry_msgs)

add_executable(velocity_smoother src/velocity_smoother.cpp)
ament_target_dependencies(velocity_smoother rclcpp geometry_msgs)

add_library(zlac8015d src/zlac8015d.cpp)
ament_export_targets(zlac8015d HAS_LIBRARY_TARGET)
ament_target_dependencies(zlac8015d rclcpp)

install(
  DIRECTORY include
  DESTINATION include
)

# install the executable in the lib folder to make it detectable through setup.bash
install(TARGETS
  run_zlac
  velocity_smoother
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  TARGETS zlac8015d
  EXPORT zlac8015d
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
