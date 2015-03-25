project(PoseToNode)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

find_package(HAL 0.1 QUIET)
find_package(Node QUIET)
find_package(PbMsgs 2.0 QUIET)
find_package(GFlags QUIET)
find_package(MINIGLOG QUIET)

if(APPLE AND NOT BUILD_SHARED_LIBS)
  set(LINK_WHOLE_ARCHIVE_FLAG -Wl,-all_load)
elseif(ANDROID)
  set(LINK_WHOLE_ARCHIVE_FLAG -Wl,--whole-archive)
  set(NO_LINK_WHOLE_ARCHIVE_FLAG -Wl,--no-whole-archive)
endif()

if(HAL_FOUND AND PbMsgs_FOUND AND MINIGLOG_FOUND AND Node_FOUND)
  add_executable( ${PROJECT_NAME} main.cpp )

  include_directories(${GFLAGS_INCLUDE_DIRS} ${HAL_INCLUDE_DIRS} ${Node_INCLUDE_DIRS}
    ${PbMsgs_INCLUDE_DIRS} ${MINIGLOG_INCLUDE_DIRS} ${NODE_INCLUDE_DIRS})
  target_link_libraries( ${PROJECT_NAME}
    ${GFLAGS_LIBRARIES} 
    ${LINK_WHOLE_ARCHIVE_FLAG}
    ${HAL_LIBRARIES}
    ${NO_LINK_WHOLE_ARCHIVE_FLAG}
    ${Node_LIBRARIES}
    ${PbMsgs_LIBRARIES}
    ${MINIGLOG_LIBRARIES}
    ${NODE_LIBRARIES}
    dl
  )

else(HAL_FOUND AND PbMsgs_FOUND AND GFlags_FOUND)
  message("Skipping ${PROJECT_NAME} because some dependency is not met:")
  message("${HAL_FOUND} ${PbMsgs_FOUND} ${GFlags_FOUND} ${MINIGLOG_FOUND}")
endif()
