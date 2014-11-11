set( BUILD_NodeCam true CACHE BOOL "Build NodeCam driver" )

if(BUILD_NodeCam)
  find_package(Node REQUIRED)
  find_package(MINIGLOG REQUIRED)

  if(Node_FOUND)
    add_to_hal_include_dirs(${Node_INCLUDE_DIRS} ${MINIGLOG_INCLUDE_DIRS})
    add_to_hal_libraries(${Node_LIBRARIES} ${MINIGLOG_LIBRARIES})
    add_to_hal_sources(NodeCamDriver.h NodeCamDriver.cpp NodeCamFactory.cpp)
  else(Node_FOUND)
    message("Node not found. NodeCam driver cannot be built")
  endif()
endif()