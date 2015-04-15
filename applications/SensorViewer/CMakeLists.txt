project(SensorViewer)

cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

find_package(Pangolin 0.1 QUIET)
find_package(OpenCV2 QUIET)

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})

if(APPLE AND NOT BUILD_SHARED_LIBS)
  set(LINK_WHOLE_ARCHIVE_FLAG -Wl,-all_load)
elseif(ANDROID)
  set(LINK_WHOLE_ARCHIVE_FLAG -Wl,--whole-archive)
  set(NO_LINK_WHOLE_ARCHIVE_FLAG -Wl,--no-whole-archive)
endif()

if(Pangolin_FOUND)
  add_executable( SensorViewer main.cpp )

  include_directories( ${HAL_DIR} )
  include_directories( ${HAL_INCLUDE_DIRS} )
  include_directories( ${Pangolin_INCLUDE_DIRS} )
  target_link_libraries(SensorViewer
    ${Pangolin_LIBRARIES}
    ${LINK_WHOLE_ARCHIVE_FLAG}
    ${HAL_LIBRARIES}
    ${NO_LINK_WHOLE_ARCHIVE_FLAG}
    ${PROTOBUF_LIBRARY}
    hal
     dl
     messages)
endif()
