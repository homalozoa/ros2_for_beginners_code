cmake_minimum_required(VERSION 3.8)
project(ch5_action_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ch5_action_interfaces REQUIRED)

set(ROS_DISTRO $ENV{ROS_DISTRO})
if(${ROS_DISTRO} STREQUAL "foxy")
  add_compile_definitions(FOXY_ACTION_API)
endif()

set(executable_self self_action)
set(executable_wrapped wrapped_action)

set(dependencies
  rclcpp
  rclcpp_action
  ch5_action_interfaces
)

include_directories(include)

add_executable(${executable_self}
  src/self_action.cpp
)

add_executable(${executable_wrapped}
  src/wrapped_action.cpp
)

set(dependency_targets
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ch5_action_interfaces::ch5_action_interfaces__rosidl_generator_cpp
  ch5_action_interfaces::ch5_action_interfaces__rosidl_typesupport_cpp
)
target_link_libraries(${executable_self} PRIVATE ${dependency_targets})
target_link_libraries(${executable_wrapped} PRIVATE ${dependency_targets})

install(
  TARGETS ${executable_self}
  TARGETS ${executable_wrapped}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
