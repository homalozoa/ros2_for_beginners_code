cmake_minimum_required(VERSION 3.8)
project(ch4_topic_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(executable_pub pub_node)
set(executable_sub sub_node)
set(executable_self self_node)

set(dependencies
  rclcpp
)

add_executable(${executable_pub}
  src/pub_node.cpp
)

add_executable(${executable_sub}
  src/sub_node.cpp
)

add_executable(${executable_self}
  src/self_node.cpp
)

ament_target_dependencies(${executable_pub}
  ${dependencies}
)

ament_target_dependencies(${executable_sub}
  ${dependencies}
)

ament_target_dependencies(${executable_self}
  ${dependencies}
)

install(
  TARGETS ${executable_pub}
  TARGETS ${executable_sub}
  TARGETS ${executable_self}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
