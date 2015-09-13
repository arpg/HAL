set(BUILD_NodeSimCam false CACHE BOOL force)

find_package(Node QUIET)
if(BUILD_NodeSimCam AND Node_FOUND)

  message( STATUS "HAL: building 'NodeSimCam' camera driver.")
  add_to_hal_include_dirs(${Node_INCLUDE_DIRS})
  add_to_hal_libraries(${Node_LIBRARIES})
  add_to_hal_sources(NodeSimCamDriver.h NodeSimCamDriver.cpp NodeSimCamFactory.cpp)
endif()
