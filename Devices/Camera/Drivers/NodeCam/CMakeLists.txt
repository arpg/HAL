# 1) Build the library:
set( DRIVER_SOURCES NodeCamDriver.cpp )
set( DRIVER_HEADERS NodeCamDriver.h )
add_library( NodeCamDriver ${DRIVER_SOURCES} ${DRIVER_HEADERS} )


# 2a) include dirs
include_directories( ${ZeroMQ_INCLUDE_DIRS} )
include_directories( ${PROTOBUF_INCLUDE_DIRS} )

# 2b) Tell RPG about this library:
add_to_rpg_libraries( NodeCamDriver )

list( APPEND RPG_LIBRARIES ${ZeroMQ_LIBRARIES} ${PROTOBUF_LIBRARIES} )
set( RPG_LIBRARIES ${RPG_LIBRARIES} CACHE INTERNAL "" FORCE )

add_to_rpg_link_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# 3) And make sure we are able to install it correctly:
install( TARGETS NodeCamDriver ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install( FILES ${DRIVER_HEADERS}
         DESTINATION
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/NodeCam )
