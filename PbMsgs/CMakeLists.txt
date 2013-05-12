project( PbMsgs )

cmake_minimum_required(VERSION 2.8)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall  -Werror")

IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
ENDIF()

# Add to module path, so we can find our cmake modules
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )


find_package( Eigen3 REQUIRED )
find_package( Protobuf REQUIRED )
find_package( Sophus REQUIRED )


find_package( OpenCV2 QUIET )
if( OpenCV2_FOUND )
    add_definitions( -DHAVE_OPENCV )
endif()

set( _PROTO_SRCS
    Messages.proto
    Camera.proto
    Image.proto
    Imu.proto
    Matrix.proto
    PoseSensor.proto
    Pose.proto
)

PROTOBUF_GENERATE_CPP( PROTO_SRCS PROTO_HDRS ${_PROTO_SRCS} )

set( _INCLUDE_DIRS
        ${PROTOBUF_INCLUDE_DIR}
        ${Sophus_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        )

include_directories( ${_INCLUDE_DIRS} )

# expose these for third parties
set( PbMsgs_INCLUDE_DIRS ${_INCLUDE_DIRS}  CACHE INTERNAL "" FORCE )

set( SRCS
    ${PROTO_SRCS}
    Logger.cpp
   )

set( HDRS
    ${PROTO_HDRS}
    Logger.h
    Image.h
    Matrix.h
    Pose.h
   )


add_library( PbMsgs ${SRCS} ${HDRS} )

get_target_property( _LIBRARY PbMsgs LOCATION )
set( PbMsgs_LIBRARIES ${_LIBRARY} ${PROTOBUF_LIBRARIES} CACHE INTERNAL "" FORCE )


# Create the *Config.cmake file for the build tree.
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/PbMsgsConfig.cmake.in
            ${CMAKE_CURRENT_BINARY_DIR}/PbMsgsConfig.cmake @ONLY IMMEDIATE )

# Add module to CMake package registery.
export( PACKAGE PbMsgs )
