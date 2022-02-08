cmake_minimum_required(VERSION 3.8)
project(cpp_chatter)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(executable_chatter chatter)
set(executable_setdomain setdomain)
set(library_name ${executable_name}_core)
set(dependencies
  rclcpp
  rclcpp_lifecycle
)

include_directories(include)

add_library(${library_name} SHARED
  src/chatter.cpp
  src/lifecycle_chatter.cpp
)

add_executable(${executable_chatter}
  src/runchatter.cpp
)

add_executable(${executable_setdomain}
  src/setdomain.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${executable_chatter}
  ${library_name}
)

target_link_libraries(${executable_setdomain}
  ${library_name}
)

install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_chatter}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${executable_setdomain}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
