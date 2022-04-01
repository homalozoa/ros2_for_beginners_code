cmake_minimum_required(VERSION 3.8)
project(ch6_bag_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_cpp REQUIRED)

set(ROS_DISTRO $ENV{ROS_DISTRO})
if(${ROS_DISTRO} STREQUAL "foxy" OR ${ROS_DISTRO} STREQUAL "galactic")
  add_compile_definitions(DEPRECATED_BAG_API)
endif()

set(executable_data pubdata)
set(executable_bag operatebag)

set(dependencies
  rclcpp
  rcpputils
  rcutils
  rosbag2_cpp
)

add_executable(${executable_data}
  src/pubdata.cpp
)

add_executable(${executable_bag}
  src/operatebag.cpp
)

ament_target_dependencies(${executable_data}
  ${dependencies}
)

ament_target_dependencies(${executable_bag}
  ${dependencies}
)

install(
  TARGETS ${executable_data}
  TARGETS ${executable_bag}
  DESTINATION lib/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
