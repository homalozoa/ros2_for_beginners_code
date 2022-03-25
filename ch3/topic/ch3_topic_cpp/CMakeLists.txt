cmake_minimum_required(VERSION 3.8)
project(ch3_topic_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(executable_pub pub_node)
set(executable_sub sub_node)
set(executable_self self_node)
set(executable_intra intra_nodes)
set(executable_loanp loan_pub)
set(executable_stat  stat_sub)

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

add_executable(${executable_intra}
  src/intra_nodes.cpp
)

add_executable(${executable_loanp}
  src/loan_pub.cpp
)

add_executable(${executable_stat}
  src/sub_statistics.cpp
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

ament_target_dependencies(${executable_intra}
  ${dependencies}
)

ament_target_dependencies(${executable_loanp}
  ${dependencies}
)

ament_target_dependencies(${executable_stat}
  ${dependencies}
)

install(
  TARGETS ${executable_pub}
  TARGETS ${executable_sub}
  TARGETS ${executable_self}
  TARGETS ${executable_intra}
  TARGETS ${executable_loanp}
  TARGETS ${executable_stat}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
