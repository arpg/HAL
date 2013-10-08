cmake_minimum_required( VERSION 2.8 )

project( Node )

if(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "-DOS_OBJECT_USE_OBJC=0")
endif()

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-unused-parameter")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter")

# Add to module path, so we can find our cmake modules
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )

find_package( ZeroMQ REQUIRED )
find_package( Protobuf REQUIRED )
find_package( Boost COMPONENTS system thread REQUIRED )

if( NOT APPLE )
  find_package( DNSSD REQUIRED )
  find_package( Avahi REQUIRED )
endif( NOT APPLE )


set(PROJ_INCLUDE_DIRS
  ${CMAKE_BINARY_DIR}
  ${CMAKE_SOURCE_DIR}/..
  ${ZeroMQ_INCLUDE_DIRS}
  ${Avahi_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
  ${DNSSD_INCLUDE_DIRS}
  ${Node_SOURCE_DIR}/..
  ${Node_BINARY_DIR}
  ${Node_SOURCE_DIR}
  )

set(PROJ_LIBRARIES
  ${PROTOBUF_LIBRARY}
  ${ZeroMQ_LIBRARIES}
  ${AVAHI_LIBRARIES}
  ${Boost_LIBRARIES}
  ${DNSSD_LIBRARIES}
  )

include_directories(${PROJ_INCLUDE_DIRS})

PROTOBUF_GENERATE_CPP( PROTO_HDRS PROTO_SRCS NodeMessages.proto )

set( NODE_HDRS Node.h PrintMessage.h ZeroConf.h ${PROTO_HDRS} )
set( NODE_SRCS Node.cpp PrintMessage.cpp ${PROTO_SRCS} )

add_library( node ${NODE_HDRS} ${NODE_SRCS} )
target_link_libraries(node ${PROJ_LIBRARIES})

# Create the NODEConfig.cmake file for the build tree.
set(EXPORT_NODE_INC
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PROTOBUF_INCLUDE_DIRS}
  ${ZeroMQ_INCLUDE_DIRS}
  )

configure_file( ${Node_SOURCE_DIR}/NODEConfig.cmake.in
  ${Node_BINARY_DIR}/NODEConfig.cmake @ONLY)

# Add module to CMake package registery.
export( TARGETS node
  FILE "${CMAKE_CURRENT_BINARY_DIR}/NODETargets.cmake")

export(PACKAGE NODE)
add_subdirectory(Examples)
