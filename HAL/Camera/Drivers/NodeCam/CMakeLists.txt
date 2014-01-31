cmake_minimum_required(VERSION 2.8)

set(BUILD_NodeCam false CACHE BOOL force)

if(BUILD_NodeCam)
  find_package( NODE REQUIRED )

  if(NODE_FOUND)
    message( "Building NodeCam" )
    find_package( Protobuf REQUIRED )
    find_package( PbMsgs 0.1 REQUIRED )
    find_package( Boost COMPONENTS system filesystem REQUIRED )

    # include dirs
    link_libraries( ${PROTOBUF_LIBRARIES} ${NODE_LIBRARIES} ${PbMsgs_LIBRARIES} )

    include_directories( ${NODE_INCLUDE_DIRS} )
    include_directories( ${PbMsgs_INCLUDE_DIRS}  )
    include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
    INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

    set( DRIVER_SOURCES NodeCamDriver.cpp )
    set( DRIVER_HEADERS NodeCamDriver.h )
    #add_library( NodeCam ${PROTO_HDRS} ${DRIVER_HEADERS} ${PROTO_SRCS} ${DRIVER_SOURCES} )
    #target_link_libraries( NodeCam ${Boost_FILESYSTEM_LIBRARY} )
    #target_link_libraries( NodeCam ${Boost_SYSTEM_LIBRARY} )

    ## Tell HAL about this library:
    add_to_hal_include_dirs( ${NODE_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} ${PROTOBUF_INCLUDE_DIRS} )
    #add_to_hal_libraries( NodeCam ${NODE_LIBRARIES})
    add_to_hal_libraries( ${NODE_LIBRARIES})
    add_to_hal_sources( NodeCamDriver.h NodeCamDriver.cpp NodeCamFactory.cpp )
  endif()
endif()
