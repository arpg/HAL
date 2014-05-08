project(logtool)
cmake_minimum_required(VERSION 2.8)

include(def_executable)

find_package(PbMsgs)
find_package(GFlags)
find_package(MINIGLOG)

include_directories(
  ${PbMsgs_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIR}
  ${MINIGLOG_INCLUDE_DIRS}
  )

def_executable(logtool
  SOURCES logtool.cpp
  DEPENDS pbmsgs
  LINK_LIBS ${GFLAGS_LIBRARIES} ${MINIGLOG_LIBRARIES}
  CONDITIONS GFLAGS_FOUND MINIGLOG_FOUND
  PACKAGE PbMsgs
  )