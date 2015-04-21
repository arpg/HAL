cmake_minimum_required(VERSION 2.8)
project(Messages)
set(VERSION_MAJOR 2)
set(VERSION_MINOR 0)
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR})

include(install_package)

string( TOLOWER ${PROJECT_NAME} LIBRARY_NAME )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall  -Werror")


#######################################################
## Find required dependencies

find_package( Eigen3 REQUIRED )
find_package( Protobuf REQUIRED )
find_package( Sophus REQUIRED )
find_package( GLog REQUIRED )
find_package( TinyXML2 )
find_package( OpenCV2 )
if( OpenCV2_FOUND )
    add_definitions( -DHAVE_OPENCV )
    list(APPEND LINK_LIBS ${OpenCV2_LIBRARIES})
    list(APPEND USER_INC ${OpenCV2_INCLUDE_DIRS})
else()
    message(STATUS "OpenCV2 not found")
endif()

list( APPEND LIB_INC_DIR "${CMAKE_CURRENT_BINARY_DIR}/..;${CMAKE_CURRENT_SOURCE_DIR}/.." )
list( APPEND USER_INC   ${CMAKE_CURRENT_BINARY_DIR} )
list( APPEND USER_INC   ${CMAKE_CURRENT_SOURCE_DIR} )
list( APPEND USER_INC   ${EIGEN3_INCLUDE_DIR} )
list( APPEND USER_INC   ${Sophus_INCLUDE_DIRS} )
list( APPEND USER_INC   ${PROTOBUF_INCLUDE_DIR} )
list( APPEND USER_INC   ${GLog_INCLUDE_DIRS} )
list( APPEND LINK_LIBS  ${PROTOBUF_LIBRARIES} )
list( APPEND LINK_LIBS  ${GLog_LIBRARIES})

if(TinyXML2_FOUND)
  list(APPEND LINK_LIBS  ${TinyXML2_LIBRARIES})
  list(APPEND USER_INC   ${TinyXML2_INCLUDE_DIRS})
endif()

include_directories( ${LIB_INC_DIR} )
include_directories( ${USER_INC} )

if(ANDROID)
    # Override to use hosts protoc compiler
    unset(PROTOBUF_PROTOC_EXECUTABLE CACHE)
    find_host_package(Protobuf REQUIRED)
endif()

#######################################################
## Sources

set( _PROTO_SRCS
    # aux
    Matrix.proto

    # header files
    Header.proto
    CameraModel.proto

    # message files
    Messages.proto
    Camera.proto
    Image.proto
    Imu.proto
    Pose.proto
    Encoder.proto
    Lidar.proto
    NodeCamMessage.proto
    NodeCar.proto
    CarPlanner.proto
    SceneGraphShapes.proto
    SimMessages.proto
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${_PROTO_SRCS})

set(SRCS
    ${PROTO_SRCS}
    Logger.cpp
    Reader.cpp
    Image.cpp
   )

set(HDRS
    ${PROTO_HDRS}
    Logger.h
    Reader.h
    Image.h
    Matrix.h
    Pose.h
   )

if(BUILD_Velodyne)
    set(SRCS ${SRCS} Velodyne.cpp)
    set(HDRS ${HDRS} Velodyne.h)
endif()

#######################################################
## Create configure file for inclusion in library

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/config.h"
  IMMEDIATE
)

add_library( messages ${SRCS} ${HDRS} )
target_link_libraries( messages ${LINK_LIBS} )

install_package(
    PKG_NAME Messages
    LIB_NAME messages
    VERSION ${VERSION}
    DESCRIPTION "Protobuf Messages handler"
    INSTALL_HEADERS ${HDRS} ${CMAKE_CURRENT_BINARY_DIR}/config.h
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/Messages/
    INCLUDE_DIRS ${LIB_INC_DIR} ${USER_INC}
    LINK_LIBS ${LINK_LIBS}
    )

