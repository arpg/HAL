project(gps2cartesian)
cmake_minimum_required(VERSION 2.8)

include(def_executable)

find_package(GeoCon)
find_package(HAL)
find_package(PbMsgs)
find_package(GFlags)
find_package(MINIGLOG)

include_directories(
  ${HAL_INCLUDE_DIRS}
  ${PbMsgs_INCLUDE_DIRS}
  ${GeoCon_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIR}
  ${MINIGLOG_INCLUDE_DIRS}
  )

set(LINK_LIBS
  ${HAL_LIBRARIES}
  ${PbMsgs_LIBRARIES}
  ${GeoCon_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${MINIGLOG_LIBRARIES}
  )

set(CONDITIONS
  HAL_FOUND
  PbMsgs_FOUND
  GeoCon_FOUND
  GFLAGS_FOUND
  MINIGLOG_FOUND
  )

def_executable(gps2cartesian
  SOURCES gps2cartesian.cpp
  LINK_LIBS ${LINK_LIBS}
  CONDITIONS ${CONDITIONS}
  )