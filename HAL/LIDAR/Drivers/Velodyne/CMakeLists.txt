list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(TinyXML2)

if(TinyXML2_FOUND)
  set(should_build ON)
else()
  set(should_build OFF)
endif()

set(BUILD_Velodyne ${should_build} CACHE BOOL "Build the Velodyne LIDAR driver")
if(BUILD_Velodyne)
  add_to_hal_sources(VelodyneDriver.h VelodyneDriver.cpp VelodyneFactory.cpp)
  add_to_hal_include_dirs(${TinyXML2_INCLUDE_DIRS})
endif()
