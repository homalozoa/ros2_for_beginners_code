cmake_minimum_required(VERSION 3.8)
project(ch5_tf2_cpp)

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

set(dependency_targets
  geometry_msgs::geometry_msgs__rosidl_generator_cpp
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
)
target_link_libraries(${executable_static} PRIVATE ${dependency_targets})
target_link_libraries(${executable_dynamic} PRIVATE ${dependency_targets})
target_link_libraries(${executable_listener} PRIVATE ${dependency_targets})

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
