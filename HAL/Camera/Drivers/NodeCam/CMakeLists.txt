find_package(Node QUIET)
find_package(GLog QUIET)
find_package(ZeroMQPP QUIET)


if(Node_FOUND AND GLog_FOUND AND ZeroMQPP_FOUND)

  set( BUILD_NodeCam true CACHE BOOL "Build NodeCam driver" )

  if(BUILD_NodeCam)
    add_to_hal_include_dirs(${Node_INCLUDE_DIRS} ${GLog_INCLUDE_DIRS} ${ZeroMQPP_INCLUDE_DIRS})
    add_to_hal_libraries(${Node_LIBRARIES} ${GLog_LIBRARIES} ${ZeroMQPP_LIBRARIES})
    add_to_hal_sources(NodeCamDriver.h NodeCamDriver.cpp NodeCamFactory.cpp)
  endif()
endif()
