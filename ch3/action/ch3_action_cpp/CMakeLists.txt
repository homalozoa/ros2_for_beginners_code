cmake_minimum_required(VERSION 3.8)
project(ch3_action_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ch3_action_interfaces REQUIRED)

set(executable_selfnode self_node)

set(dependencies
  rclcpp
  rclcpp_action
  ch3_action_interfaces
)

include_directories(include)

add_executable(${executable_selfnode}
  src/single_node.cpp
)

ament_target_dependencies(
  ${executable_selfnode}
  ${dependencies}
)

install(
  TARGETS ${executable_selfnode}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
