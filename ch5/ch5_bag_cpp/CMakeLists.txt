cmake_minimum_required(VERSION 3.8)
project(ch5_bag_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(executable_data pubdata)

set(dependencies
  rclcpp
)

add_executable(${executable_data}
  src/pubdata.cpp
)

ament_target_dependencies(${executable_data}
  ${dependencies}
)

install(
  TARGETS ${executable_data}
  DESTINATION lib/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
