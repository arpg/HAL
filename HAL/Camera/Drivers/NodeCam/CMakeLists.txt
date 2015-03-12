find_package(Node QUIET)
find_package(MINIGLOG QUIET)
find_package(ZeroMQPP QUIET)


if(Node_FOUND AND MINIGLOG_FOUND AND ZeroMQPP_FOUND)

  set( BUILD_NodeCam true CACHE BOOL "Build NodeCam driver" )

  if(BUILD_NodeCam)
    add_to_hal_include_dirs(${Node_INCLUDE_DIRS} ${MINIGLOG_INCLUDE_DIRS} ${ZeroMQPP_INCLUDE_DIRS})
    add_to_hal_libraries(${Node_LIBRARIES} ${MINIGLOG_LIBRARIES} ${ZeroMQPP_LIBRARIES})
    add_to_hal_sources(NodeCamDriver.h NodeCamDriver.cpp NodeCamFactory.cpp)
  endif()
endif()
