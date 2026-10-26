cmake_minimum_required(VERSION 3.10)
project(go2_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FootContacts.msg"
  "msg/GaitSchedule.msg"
  "msg/FootTrajectory.msg"
  "msg/ContactModeHorizon.msg"
  "msg/ModelObservation.msg"
  "msg/WbcSolution.msg"
  "msg/MrtReference.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
