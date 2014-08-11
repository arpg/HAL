project(logtool)
cmake_minimum_required(VERSION 2.8)

include(def_executable)

find_package(PbMsgs)
find_package(GFlags)
find_package(MINIGLOG)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall  -Werror")

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
