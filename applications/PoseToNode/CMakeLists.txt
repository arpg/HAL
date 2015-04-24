project(PoseToNode)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

find_package(Node REQUIRED)
find_package(GFlags REQUIRED)
find_package(GLog REQUIRED)

if(APPLE AND NOT BUILD_SHARED_LIBS)
  set(LINK_WHOLE_ARCHIVE_FLAG -Wl,-all_load)
elseif(ANDROID)
  set(LINK_WHOLE_ARCHIVE_FLAG -Wl,--whole-archive)
  set(NO_LINK_WHOLE_ARCHIVE_FLAG -Wl,--no-whole-archive)
endif()

if(GLOG_FOUND AND Node_FOUND)
  add_executable( ${PROJECT_NAME} main.cpp )

  include_directories(${GFLAGS_INCLUDE_DIRS} ${HAL_INCLUDE_DIRS} ${Node_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS})
  target_link_libraries( ${PROJECT_NAME}
    ${GFLAGS_LIBRARIES} 
    ${LINK_WHOLE_ARCHIVE_FLAG}
    ${HAL_LIBRARIES}
    ${NO_LINK_WHOLE_ARCHIVE_FLAG}
    ${GLOG_LIBRARIES}
    ${Node_LIBRARIES}
    dl
  )

else(GFlags_FOUND)
  message("Skipping ${PROJECT_NAME} because some dependency is not met:")
  message("${HAL_FOUND} ${GFlags_FOUND} ${GLOG_FOUND}")
endif()
