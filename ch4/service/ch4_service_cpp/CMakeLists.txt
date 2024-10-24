cmake_minimum_required(VERSION 3.8)
project(ch4_service_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(ROS_DISTRO $ENV{ROS_DISTRO})
if(${ROS_DISTRO} STREQUAL "rolling")
  add_compile_definitions(NEW_SERVICE_TEMPLATE)
endif()

set(executable_selfservice self_exec)
set(executable_selfnode self_node)

set(dependencies
  rclcpp
)

add_executable(${executable_selfservice}
  src/self_service.cpp
)

add_executable(${executable_selfnode}
  src/single_node.cpp
)

ament_target_dependencies(
  ${executable_selfservice}
  ${dependencies}
)

ament_target_dependencies(
  ${executable_selfnode}
  ${dependencies}
)

install(
  TARGETS ${executable_selfservice}
  TARGETS ${executable_selfnode}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
