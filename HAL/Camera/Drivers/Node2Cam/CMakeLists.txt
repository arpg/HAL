cmake_minimum_required(VERSION 2.8)

set( BUILD_Node2Cam false CACHE BOOL force )

find_package( NODE QUIET )

if(NODE_FOUND AND BUILD_Node2Cam)
    find_package( Protobuf REQUIRED )
    find_package( PbMsgs 0.1 REQUIRED )

    # include dirs
    link_libraries( ${PROTOBUF_LIBRARIES} ${NODE_LIBRARIES} ${PbMsgs_LIBRARIES} )

    include_directories( ${NODE_INCLUDE_DIRS} )
    include_directories( ${PbMsgs_INCLUDE_DIRS}  )
    include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

    set( DRIVER_SOURCES Node2CamDriver.cpp )
    set( DRIVER_HEADERS Node2CamDriver.h )
    add_library( Node2Cam ${PROTO_HDRS} ${DRIVER_HEADERS} ${PROTO_SRCS} ${DRIVER_SOURCES}  )

    ## Tell HAL about this library:
    add_to_hal_include_dirs( ${NODE_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} ${PROTOBUF_INCLUDE_DIRS} )
    add_to_hal_libraries( Node2Cam ${NODE_LIBRARIES})
    add_to_hal_sources( Node2CamDriver.h Node2CamDriver.cpp Node2CamFactory.cpp )
endif()


