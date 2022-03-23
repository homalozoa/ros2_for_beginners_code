cmake_minimum_required(VERSION 3.8)
project(ch5_unittest_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(lib_monkeyraw monkeyraw)
set(test_monkeyraw monkeyrawtest)

include_directories(include)

add_library(${lib_monkeyraw} SHARED
  src/monkey_raw.cpp
)

install(
  TARGETS ${lib_monkeyraw}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${test_monkeyraw}
    test/raw_test.cpp
    TIMEOUT 20
  )
  target_link_libraries(${test_monkeyraw}
    ${lib_monkeyraw}
  )
endif()

ament_export_libraries(${lib_monkeyraw})
ament_export_dependencies(${dependencies})
ament_package()
