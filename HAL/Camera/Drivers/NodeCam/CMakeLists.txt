find_package(Node QUIET)


if(Node_FOUND)
  find_package(MINIGLOG REQUIRED)
  find_package(ZeroMQPP REQUIRED)

  set( BUILD_NodeCam true CACHE BOOL "Build NodeCam driver" )

  if(BUILD_NodeCam)
    add_to_hal_include_dirs(${Node_INCLUDE_DIRS} ${MINIGLOG_INCLUDE_DIRS} ${ZeroMQPP_INCLUDE_DIRS})
    add_to_hal_libraries(${Node_LIBRARIES} ${MINIGLOG_LIBRARIES} ${ZeroMQPP_LIBRARIES})
    add_to_hal_sources(NodeCamDriver.h NodeCamDriver.cpp NodeCamFactory.cpp)
  endif()
endif()
