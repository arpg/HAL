if(0)
PROTOBUF_GENERATE_CPP( PROTO_HDRS PROTO_SRCS Message.proto )

# include dirs
include_directories( ${ZeroMQ_INCLUDE_DIRS} )
include_directories( ${PROTOBUF_INCLUDE_DIRS} )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# Build the library:
set( DRIVER_SOURCES NodeCamDriver.cpp )
set( DRIVER_HEADERS NodeCamDriver.h )
add_library( NodeCamDriver ${PROTO_HDRS} ${DRIVER_HEADERS} ${PROTO_SRCS} ${DRIVER_SOURCES} )

# Tell RPG about this library:
add_to_hal_libraries( NodeCamDriver ${ZeroMQ_LIBRARIES} ${PROTOBUF_LIBRARIES} )

# 3) And make sure we are able to install it correctly:
install( TARGETS NodeCamDriver
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

install( FILES ${DRIVER_HEADERS}
         DESTINATION
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/NodeCam )
endif()
