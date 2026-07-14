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

if(message_filters_VERSION VERSION_GREATER_EQUAL "7.0.0")
  target_compile_definitions(${executable_fltr} PRIVATE MESSAGE_FILTERS_HAS_RCLCPP_QOS)
endif()

target_link_libraries(${executable_fltr} PRIVATE
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  message_filters::message_filters
)

install(
  TARGETS ${executable_fltr}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
