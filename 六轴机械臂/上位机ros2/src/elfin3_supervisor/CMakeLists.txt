cmake_minimum_required(VERSION 3.8)
project(elfin3_supervisor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(elfin3_supervisor_node
  src/elfin3_supervisor_node.cpp
)
ament_target_dependencies(elfin3_supervisor_node
  control_msgs
  controller_manager_msgs
  diagnostic_msgs
  moveit_msgs
  rclcpp
  rclcpp_action
  rosgraph_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
)

install(
  TARGETS elfin3_supervisor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
