project(commander_node)
cmake_minimum_required(VERSION 2.8)

set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules ${CMAKE_MODULE_PATH} )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

option(JOYSTICK_CONTROL "Enable in order to control ninja with joystick" ON)

find_package( HAL REQUIRED )
find_package( Node REQUIRED )
find_package( Protobuf REQUIRED )
include_directories( ${CMAKE_BINARY_DIR} )
PROTOBUF_GENERATE_CPP( PROTO_SRCS PROTO_HDRS NinjaMsgs.proto )

include_directories( ${HAL_INCLUDE_DIRS} )
include_directories( ${Node_INCLUDE_DIRS} )

add_subdirectory( Gamepad )
set( HDRS ${PROTO_HDRS} JoystickHandler.h)

if(JOYSTICK_CONTROL)
		set( SRCS JoystickHandler.cpp commander_node.cpp ${PROTO_SRCS} )
else()
		set( SRCS main.cpp JoystickHandler.cpp  ${PROTO_SRCS} )
endif()

add_executable( ${PROJECT_NAME} ${HDRS} ${SRCS} )

target_link_libraries( ${PROJECT_NAME}
		${HAL_LIBRARIES}
    ${PROTOBUF_LIBRARY}
		${Node_LIBRARIES}
		${PROTOBUF_LIBRARIES}
		Gamepad
		)
