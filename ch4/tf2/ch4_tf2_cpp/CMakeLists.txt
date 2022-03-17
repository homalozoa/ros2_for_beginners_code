cmake_minimum_required(VERSION 3.8)
project(ch4_tf2_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(executable_static static_transform)
set(executable_dynamic dynamic_transform)
set(executable_listener transform_listener)

set(dependencies
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)

add_executable(${executable_static}
  src/static_transform.cpp
)

add_executable(${executable_dynamic}
  src/dynamic_transform.cpp
)

add_executable(${executable_listener}
  src/transform_listener.cpp
)

ament_target_dependencies(
  ${executable_static}
  ${dependencies}
)

ament_target_dependencies(
  ${executable_dynamic}
  ${dependencies}
)

ament_target_dependencies(
  ${executable_listener}
  ${dependencies}
)

install(
  TARGETS ${executable_static}
  TARGETS ${executable_dynamic}
  TARGETS ${executable_listener}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
