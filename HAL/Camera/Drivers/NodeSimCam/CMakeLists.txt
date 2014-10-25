set(BUILD_NodeSimCam false CACHE BOOL force)

if(BUILD_NodeSimCam)
  find_package(Node REQUIRED)
  find_package(MINIGLOG REQUIRED)

  if(Node_FOUND)
    add_to_hal_include_dirs(${Node_INCLUDE_DIRS} ${MINIGLOG_INCLUDE_DIRS})
    add_to_hal_libraries(${Node_LIBRARIES} ${MINIGLOG_LIBRARIES})
    add_to_hal_sources(NodeSimCamDriver.h NodeSimCamDriver.cpp NodeSimCamFactory.cpp)
  endif()
endif()
