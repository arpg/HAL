project(gps2cartesian)
cmake_minimum_required(VERSION 2.8)

include(def_executable)

find_package(GeoCon QUIET)
find_package(HAL QUIET)
find_package(GFlags QUIET)
find_package(GLog QUIET)

include_directories(
  ${HAL_INCLUDE_DIRS}
  ${GeoCon_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIR}
  ${GLog_INCLUDE_DIRS}
  )

link_directories(${GeoCon_LINK_DIRS})

set(LINK_LIBS
  ${HAL_LIBRARIES}
  ${GeoCon_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${GLog_LIBRARIES}
  messages )

set(CONDITIONS
  HAL_FOUND
  GeoCon_FOUND
  GFLAGS_FOUND
  GLog_FOUND
  )

list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

def_executable(gps2cartesian
  SOURCES gps2cartesian.cpp
  LINK_LIBS ${LINK_LIBS}
  CONDITIONS ${CONDITIONS}
  )
