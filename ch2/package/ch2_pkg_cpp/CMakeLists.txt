cmake_minimum_required(VERSION 3.8)
project(ch2_pkg_cpp)

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

set(executable_pkg2go pkg2go)
set(library_pkg2go ${executable_pkg2go}_core)
include_directories(include)

add_library(${library_pkg2go} SHARED
  src/pkg2go.cpp
)

add_executable(${executable_pkg2go}
  src/main.cpp
)

target_link_libraries(${executable_pkg2go}
  ${library_pkg2go}
)

install(TARGETS
  ${library_pkg2go}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_pkg2go}
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
ament_export_libraries(${library_pkg2go})
ament_package()
