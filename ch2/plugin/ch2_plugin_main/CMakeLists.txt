cmake_minimum_required(VERSION 3.8)
project(ch2_plugin_main)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ch2_plugin_base REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

set(exec_withoutnode run_plugin_withoutnode)
set(exec_withnode run_plugin_withnode)

add_executable(${exec_withoutnode} src/withoutnode.cpp)
add_executable(${exec_withnode} src/withnode.cpp)

ament_target_dependencies(${exec_withoutnode}
  ch2_plugin_base
  pluginlib
  rclcpp
)
ament_target_dependencies(${exec_withnode}
  ch2_plugin_base
  pluginlib
  rclcpp
)

install(
  TARGETS ${exec_withoutnode}
  TARGETS ${exec_withnode}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
