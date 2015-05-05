cmake_minimum_required(VERSION 2.8)

set(BUILD_NodeCar false CACHE BOOL force)

if(BUILD_NodeCar)
  find_package(Node QUIET)
  find_package(PbMsgs QUIET)

  if(Node_FOUND AND PbMsgs_FOUND)
    set(DRIVER_SOURCES NodeCarDriver.cpp)
    set(DRIVER_HEADERS NodeCarDriver.h)
    message("Node: ${Node_INCLUDE_DIRS}" )
    # add_to_hal_libraries add_to_hal_include_dirs not called as NodeCar
    # would have added Node related values.
    message("PbMsgs: ${PbMsgs_INCLUDE_DIRS}" )
    add_to_hal_sources(NodeCarDriver.h NodeCarDriver.cpp NodeCarFactory.cpp)
    add_to_hal_libraries( ${Node_LIBRARIES} )
    add_to_hal_include_dirs( ${Node_INCLUDE_DIRS} ${PbMsgs_INCLUDE_DIRS} )
  endif()

endif()
