find_package(Node QUIET)
find_package(GLog QUIET)
find_package(ZeroMQPP QUIET)

if(Node_FOUND AND GLOG_FOUND AND ZeroMQPP_FOUND)

option(BUILD_NodeCam false "Build NodeCam Driver" )

  if(BUILD_NodeCam)
    message( STATUS "HAL: building 'NodeCam' camera driver.")
    add_to_hal_include_dirs(${Node_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS} ${ZeroMQPP_INCLUDE_DIRS})
    add_to_hal_libraries(${Node_LIBRARIES} ${GLOG_LIBRARIES} ${ZeroMQPP_LIBRARIES})
    add_to_hal_sources(NodeCamDriver.h NodeCamDriver.cpp NodeCamFactory.cpp)
  endif()
endif()
