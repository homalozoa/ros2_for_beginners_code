cmake_minimum_required(VERSION 3.8)
project(ch5_v_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SE3Velocity.msg"
  "srv/GetVelocity.srv"
  "srv/SetVelocityLimit.srv"
  "action/SpeedUpTo.action")

ament_export_dependencies(rosidl_default_runtime)
ament_package()
