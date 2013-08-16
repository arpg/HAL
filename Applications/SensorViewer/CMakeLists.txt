project(SensorViewer)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

find_package(Pangolin 0.1 QUIET)
find_package(HAL 0.1 QUIET)

find_package( OpenCV2 QUIET )

if(Pangolin_FOUND AND HAL_FOUND)
    add_executable( SensorViewer main.cpp )

    include_directories( ${HAL_INCLUDE_DIRS} )
    include_directories( ${Pangolin_INCLUDE_DIRS} )
    target_link_libraries(SensorViewer ${HAL_LIBRARIES} )
    target_link_libraries(SensorViewer ${Pangolin_LIBRARIES} )

    add_dependencies( SensorViewer hal )                                              
    add_dependencies( SensorViewer PbMsgs )   
endif()
