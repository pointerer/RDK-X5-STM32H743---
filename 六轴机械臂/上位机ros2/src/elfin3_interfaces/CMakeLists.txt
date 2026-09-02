cmake_minimum_required(VERSION 3.8)
project(elfin3_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/PlanningMetrics.msg
  "msg/JogCommand.msg"
  "action/MoveJ.action"
  "action/MovePose.action"
  DEPENDENCIES action_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
