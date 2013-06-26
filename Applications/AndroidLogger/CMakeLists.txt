project(AndroidLogger) 
cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
include( AndroidUtils )

set( CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS}" )

# git://github.com/stevenlovegrove/Pangolin.git
find_package(Pangolin 0.1 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
message( STATUS "LIB: ${Pangolin_LIBRARIES}" )

find_package(SceneGraph 0.1 REQUIRED)
include_directories(${SceneGraph_INCLUDE_DIRS})
message( STATUS "LIB: ${SceneGraph_LIBRARIES}" )

## git clone ssh://robotics.gwu.edu/home/rpg/git/calibu
#find_package( Calibu 0.1 REQUIRED )
#include_directories( ${Calibu_INCLUDE_DIRS} )
#target_link_libraries(${PROJECT_NAME} ${Calibu_LIBRARIES} )
#message( STATUS "LIB: ${Calibu_LIBRARIES}" )

find_package( PbMsgs 0.1 REQUIRED )
include_directories( ${PbMsgs_INCLUDE_DIRS} )
message( STATUS "LIB: ${PbMsgs_LIBRARIES}" )

find_package( HAL 0.1 REQUIRED )
include_directories( ${HAL_INCLUDE_DIRS} )
message( STATUS "LIB: ${HAL_LIBRARIES}" )


add_executable(${PROJECT_NAME} main.cpp)


target_link_libraries(${PROJECT_NAME} ${Pangolin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${SceneGraph_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${HAL_LIBRARIES} )
target_link_libraries(${PROJECT_NAME} ${PbMsgs_LIBRARIES} )


