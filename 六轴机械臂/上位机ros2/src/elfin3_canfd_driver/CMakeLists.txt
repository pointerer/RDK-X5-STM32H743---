cmake_minimum_required(VERSION 3.8)
project(elfin3_canfd_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)

add_library(elfin3_canfd_transport SHARED
  src/can_fd_transport.cpp
  src/csp_scheduler.cpp
)
target_include_directories(elfin3_canfd_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(elfin3_canfd_transport PUBLIC Threads::Threads)

add_library(elfin3_canfd_system SHARED
  src/elfin3_canfd_system.cpp
)
target_include_directories(elfin3_canfd_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(elfin3_canfd_system
  elfin3_canfd_transport
  Threads::Threads
)
ament_target_dependencies(elfin3_canfd_system
  diagnostic_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
)
pluginlib_export_plugin_description_file(
  hardware_interface elfin3_canfd_hardware.xml
)

add_executable(elfin3_canfd_driver_node
  src/driver_node.cpp
)
target_include_directories(elfin3_canfd_driver_node PRIVATE include)
target_link_libraries(elfin3_canfd_driver_node elfin3_canfd_transport)
ament_target_dependencies(elfin3_canfd_driver_node
  diagnostic_msgs
  rclcpp
  sensor_msgs
  std_srvs
)

install(TARGETS elfin3_canfd_transport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS elfin3_canfd_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS elfin3_canfd_driver_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS scripts/setup_can0.sh
  scripts/setup_vcan.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(protocol_test test/protocol_test.cpp)
  target_include_directories(protocol_test PRIVATE include)
  target_link_libraries(protocol_test elfin3_canfd_transport)

  ament_add_gtest(vcan_transport_test test/vcan_transport_test.cpp)
  target_include_directories(vcan_transport_test PRIVATE include)
  target_link_libraries(vcan_transport_test elfin3_canfd_transport)
endif()

ament_export_include_directories(include)
ament_export_libraries(elfin3_canfd_transport)
ament_export_libraries(elfin3_canfd_system)
ament_export_dependencies(
  diagnostic_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
)

ament_package()
