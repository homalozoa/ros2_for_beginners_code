cmake_minimum_required(VERSION 3.8)
project(ch2_node_cpp)

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

set(executable_node2go node2go)
set(executable_node2go2 node2go2)
set(executable_multinode multinode)
set(executable_multilifecyclenode multilifecyclenode)
set(executable_global_arg globalarg)
set(library_node2go nodelib)
set(library_getid getid)
set(library_lifecyclenode lifecyclenode2go)
set(dependencies
  rclcpp
  rclcpp_lifecycle
)

include_directories(include)

add_library(${library_node2go} SHARED
  src/node2go.cpp
)
add_library(${library_getid} SHARED
  src/getid.cpp
)
add_library(${library_lifecyclenode} SHARED
  src/lifecyclenode2go.cpp
)

add_executable(${executable_node2go}
  src/main.cpp
)
add_executable(${executable_node2go2}
  src/main2.cpp
)
add_executable(${executable_multinode}
  src/multinode.cpp
)
add_executable(${executable_multilifecyclenode}
  src/multilifecyclenode.cpp
)
add_executable(${executable_global_arg}
  src/global_arg.cpp
)

set(dependency_targets
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

target_link_libraries(${library_node2go} PUBLIC ${dependency_targets})
target_link_libraries(${library_getid} PUBLIC ${dependency_targets})
target_link_libraries(${library_lifecyclenode} PUBLIC ${dependency_targets})
target_link_libraries(${executable_node2go} PRIVATE ${dependency_targets})
target_link_libraries(${executable_node2go2} PRIVATE ${dependency_targets})
target_link_libraries(${executable_multinode} PRIVATE ${dependency_targets})
target_link_libraries(${executable_multilifecyclenode} PRIVATE ${dependency_targets})
target_link_libraries(${executable_global_arg} PRIVATE ${dependency_targets})

target_link_libraries(${executable_node2go}
  PRIVATE ${library_node2go}
)
target_link_libraries(${executable_node2go2}
  PRIVATE ${library_node2go}
)
target_link_libraries(${executable_multinode}
  PRIVATE ${library_getid}
)
target_link_libraries(${executable_multilifecyclenode}
  PRIVATE ${library_lifecyclenode}
)

install(TARGETS
  ${library_node2go}
  ${library_getid}
  ${library_lifecyclenode}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS ${executable_node2go}
  TARGETS ${executable_node2go2}
  TARGETS ${executable_multinode}
  TARGETS ${executable_multilifecyclenode}
  TARGETS ${executable_global_arg}
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
ament_export_libraries(${library_node2go} ${library_getid} ${library_lifecyclenode})
ament_export_dependencies(${dependencies})
ament_package()
