cmake_minimum_required(VERSION 3.8)
project(ch2_plugin_alpha)

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

set(lib_alpha plugins_alpha)
set(dependencies
  ch2_plugin_base
  pluginlib
  rclcpp
)

add_library(${lib_alpha} SHARED
  src/pluginalpha.cpp
)

ament_target_dependencies(${lib_alpha}
  ${dependencies}
)

install(
  TARGETS ${lib_alpha}
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(${lib_alpha})
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(ch2_plugin_base alpha_defines.xml)
ament_package()
