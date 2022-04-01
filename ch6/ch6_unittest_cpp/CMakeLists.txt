cmake_minimum_required(VERSION 3.8)
project(ch6_unittest_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(lib_monkey monkey)
set(test_monkey monkeytest)

set(dependencies
  rclcpp
)

include_directories(include)

add_library(${lib_monkey} SHARED
  src/monkey.cpp
)

ament_target_dependencies(${lib_monkey}
  ${dependencies}
)

install(
  TARGETS ${lib_monkey}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${test_monkey}
    test/monkey_test.cpp
    TIMEOUT 20
  )
  ament_target_dependencies(${test_monkey}
    ${dependencies}
  )
  target_link_libraries(${test_monkey}
    ${lib_monkey}
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${lib_monkey})
ament_export_dependencies(${dependencies})
ament_package()
