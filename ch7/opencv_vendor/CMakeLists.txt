cmake_minimum_required(VERSION 3.8)
project(opencv_vendor)

find_package(ament_cmake REQUIRED)
find_package(OpenCV QUIET)

if(OpenCV_FOUND)
  message("OpenCV ${OpenCV_VERSION} found, skip building ${PROJECT_NAME}")
  return()
endif()

set(EXTPRJ_NAME opencv)
set(EXTPRJ_DEP opencv-contrib)
set(PKG_VER "4.2.0")
set(EXTERNAL_DOWNLOAD_LOCATION ${CMAKE_BINARY_DIR}/Download)

include(ExternalProject)

externalproject_add(${EXTPRJ_DEP}
  URL https://github.com/opencv/opencv_contrib/archive/refs/tags/${PKG_VER}.tar.gz
  URL_MD5 7f8111deb2ce3ed6c87ede8b3bf82031
  SOURCE_DIR "${EXTERNAL_DOWNLOAD_LOCATION}/opencv-contrib"
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)

externalproject_add(
  ${EXTPRJ_NAME}
  PREFIX ${EXTPRJ_NAME}
  URL https://github.com/opencv/opencv/archive/refs/tags/${PKG_VER}.tar.gz
  URL_MD5 e8cb208ce2723481408b604b480183b6
  SOURCE_DIR "${EXTERNAL_DOWNLOAD_LOCATION}/opencv"
  CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
      -DBUILD_DOCS:BOOL=OFF
      -DBUILD_EXAMPLES:BOOL=OFF
      -DBUILD_PACKAGE:BOOL=OFF
      -DBUILD_SHARED_LIBS:BOOL=ON
      -DBUILD_TESTS:BOOL=OFF
      -DOPENCV_EXTRA_MODULES_PATH=${EXTERNAL_DOWNLOAD_LOCATION}/opencv-contrib/modules
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  TIMEOUT 1200
)
add_dependencies(${EXTPRJ_NAME} ${EXTPRJ_DEP})

ament_package()
