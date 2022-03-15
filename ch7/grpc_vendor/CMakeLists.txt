cmake_minimum_required(VERSION 3.8)
project(grpc_vendor)

find_package(ament_cmake REQUIRED)
find_package(Protobuf QUIET)
find_package(gRPC CONFIG QUIET)

if(Protobuf_FOUND AND gRPC_FOUND)
  message("protobuf & gRPC found, skip building from source")
  return()
endif()

set(EXTPRJ_NAME grpc)
set(GIT_URL "https://github.com/grpc/grpc.git")
set(PKG_VER "v1.36.3")

include(ExternalProject)

externalproject_add(
  ${EXTPRJ_NAME}
  PREFIX ${EXTPRJ_NAME}
  GIT_REPOSITORY ${GIT_URL}
  GIT_TAG ${PKG_VER}
  CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  TIMEOUT 1200
)

ament_package()