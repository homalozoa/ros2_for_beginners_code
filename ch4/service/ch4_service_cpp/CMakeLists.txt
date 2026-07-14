cmake_minimum_required(VERSION 3.8)
project(ch4_service_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

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

target_link_libraries(${executable_selfservice} PRIVATE rclcpp::rclcpp)
target_link_libraries(${executable_selfnode} PRIVATE rclcpp::rclcpp)

if(rclcpp_VERSION VERSION_LESS "21.0.0")
  target_compile_definitions(${executable_selfnode} PRIVATE RCLCPP_SERVICE_USES_RMW_QOS)
endif()

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
