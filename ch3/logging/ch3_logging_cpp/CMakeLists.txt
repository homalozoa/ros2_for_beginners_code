cmake_minimum_required(VERSION 3.8)
project(ch3_logging_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(executable_test logger_test)
set(executable_stuck stuck_logger)
set(dependencies
  rclcpp
)

add_executable(${executable_test}
  src/logger_test.cpp
)
add_executable(${executable_stuck}
  src/stuck_logger.cpp
)

ament_target_dependencies(${executable_test}
  ${dependencies}
)
ament_target_dependencies(${executable_stuck}
  ${dependencies}
)

install(
  TARGETS ${executable_test}
  TARGETS ${executable_stuck}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
