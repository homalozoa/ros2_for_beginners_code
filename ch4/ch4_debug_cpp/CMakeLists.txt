cmake_minimum_required(VERSION 3.8)
project(ch4_debug_cpp)

set(CMAKE_BUILD_TYPE "Debug")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(executable_debug tracetest)

set(dependencies
  rclcpp
)

add_executable(${executable_debug}
  src/faultcode.cpp
)

ament_target_dependencies(${executable_debug}
  ${dependencies}
)

install(
  TARGETS ${executable_debug}
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
