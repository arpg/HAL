cmake_minimum_required(VERSION 2.8)
project( AndCam )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-variable -Wno-unused-parameter")

find_package( HAL 0.1 REQUIRED )
find_package( Node REQUIRED )
find_package( MINIGLOG REQUIRED )
#find_package( Calibu 0.1 REQUIRED )

include_directories( . )
include_directories( ${HAL_INCLUDE_DIRS} )
include_directories( ${NODE_INCLUDE_DIRS} )
include_directories( ${Node_INCLUDE_DIRS} )
include_directories( ${MINIGLOG_INCLUDE_DIRS} )

add_library( AndCam SHARED AndCam.cpp AndCam.h )
#add_executable( TestCam TestCam.cpp AndCam.h )

target_link_libraries( AndCam ${NODE_LIBRARIES} )
target_link_libraries( AndCam ${Node_LIBRARIES} )
target_link_libraries( AndCam ${HAL_LIBRARIES} )

#target_link_libraries( TestCam ${NODE_LIBRARIES} )
#target_link_libraries( TestCam ${Node_LIBRARIES} )
#target_link_libraries( TestCam ${HAL_LIBRARIES} )

