cmake_minimum_required(VERSION 3.8)
project(ch3_plugin_beta)

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
find_package(ch3_plugin_base REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

set(lib_beta plugins_beta)
set(dependencies
  ch3_plugin_base
  pluginlib
  rclcpp
)

add_library(${lib_beta} SHARED
  src/pluginbeta.cpp)

ament_target_dependencies(${lib_beta}
  ${dependencies}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS ${lib_beta}
  LIBRARY DESTINATION lib
)

ament_export_libraries(${lib_beta})
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(ch3_plugin_base beta_defines.xml)
ament_package()
