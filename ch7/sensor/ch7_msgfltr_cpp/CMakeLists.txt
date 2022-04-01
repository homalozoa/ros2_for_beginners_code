cmake_minimum_required(VERSION 3.8)
project(ch7_msgfltr_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)


set(executable_fltr selfilter)
set(dependencies
  rclcpp
  sensor_msgs
  message_filters
)

add_executable(${executable_fltr}
  src/selfilter.cpp
)

ament_target_dependencies(${executable_fltr}
  ${dependencies}
)

install(
  TARGETS ${executable_fltr}
  DESTINATION lib/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
