cmake_minimum_required(VERSION 3.8)
project(ch7_diagnostics_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(sensor_msgs REQUIRED)

set(executable_simpleupdater simple_updater)
set(executable_freqchecker freq_checker)

set(dependencies
  rclcpp
  diagnostic_updater
  sensor_msgs
)

add_executable(${executable_simpleupdater}
  src/simple_updater.cpp
)
add_executable(${executable_freqchecker}
  src/freq_checker.cpp
)


set(dependency_targets
  rclcpp::rclcpp
  diagnostic_updater::diagnostic_updater
  sensor_msgs::sensor_msgs_library
)
target_link_libraries(${executable_simpleupdater} PRIVATE ${dependency_targets})
target_link_libraries(${executable_freqchecker} PRIVATE ${dependency_targets})

install(
  TARGETS ${executable_simpleupdater}
  TARGETS ${executable_freqchecker}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/param/
)

install(
  DIRECTORY param/
  DESTINATION share/${PROJECT_NAME}/param/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
