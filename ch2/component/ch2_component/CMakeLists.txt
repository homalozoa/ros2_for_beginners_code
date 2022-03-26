cmake_minimum_required(VERSION 3.8)
project(ch2_component)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

set(library_components component_test)
set(dependencies
  rclcpp
  rclcpp_components
)

add_library(${library_components} SHARED
  src/com1.cpp
  src/com2.cpp
)

ament_target_dependencies(${library_components}
  ${dependencies}
)

rclcpp_components_register_node(${library_components}
  PLUGIN "ros_beginner::Component1"
  EXECUTABLE componentest1)

rclcpp_components_register_node(${library_components}
  PLUGIN "ros_beginner::Component2"
  EXECUTABLE componentest2)

rclcpp_components_register_node(${library_components}
  PLUGIN "ros_beginner::Component3"
  EXECUTABLE componentest3)

# rclcpp_components_register_nodes(${library_components}
#   "ros_beginner::Component1"
#   "ros_beginner::Component2"
#   "ros_beginner::Component3")

install(TARGETS ${library_components}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
