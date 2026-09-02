cmake_minimum_required(VERSION 3.8)
project(elfin3_jog)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(elfin3_interfaces REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(elfin3_jog_controller SHARED
  src/jog_controller.cpp
)
target_include_directories(elfin3_jog_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(elfin3_jog_controller
  controller_interface
  elfin3_interfaces
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  std_srvs
)

add_executable(elfin3_keyboard_teleop
  src/keyboard_teleop.cpp
)
ament_target_dependencies(elfin3_keyboard_teleop elfin3_interfaces rclcpp)

add_executable(elfin3_jog_mode_manager
  src/jog_mode_manager.cpp
)
ament_target_dependencies(elfin3_jog_mode_manager
  controller_manager_msgs
  elfin3_interfaces
  rclcpp
  std_msgs
  std_srvs
)

pluginlib_export_plugin_description_file(
  controller_interface elfin3_jog_plugins.xml
)

install(TARGETS elfin3_jog_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS elfin3_keyboard_teleop
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS elfin3_jog_mode_manager
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(FILES elfin3_jog_plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(jog_math_test test/jog_math_test.cpp)
  target_include_directories(jog_math_test PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_libraries(elfin3_jog_controller)
ament_export_dependencies(
  controller_interface
  controller_manager_msgs
  elfin3_interfaces
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  std_srvs
)

ament_package()
