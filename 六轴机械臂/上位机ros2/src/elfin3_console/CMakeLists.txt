cmake_minimum_required(VERSION 3.8)
project(elfin3_console)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(elfin3_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(elfin3_console_node
  src/main.cpp
  src/console_node.cpp
)
target_include_directories(elfin3_console_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(elfin3_console_node
  diagnostic_msgs
  elfin3_interfaces
  rclcpp
  rclcpp_action
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
)

install(
  TARGETS elfin3_console_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
