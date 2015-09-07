find_package(Node QUIET)
find_package(GLog QUIET)

if(GLog_FOUND AND Node_FOUND )

  option(BUILD_NodeEncoder false)

  if(BUILD_NodeEncoder)
    add_to_hal_include_dirs(${Node_INCLUDE_DIRS} ${GLog_INCLUDE_DIRS})
    add_to_hal_libraries(${Node_LIBRARIES} ${GLog_LIBRARIES})
    add_to_hal_sources(NodeEncoderDriver.h NodeEncoderDriver.cpp NodeEncoderFactory.cpp)
  endif()
elseif(NOT GLOG_FOUND)
    message("GLog not found. NodeEncoder driver cannot be built")
elseif(NOT Node_FOUND)
  message( STATUS "HAL: NOT building the NodeEncoder driver.")
endif()
