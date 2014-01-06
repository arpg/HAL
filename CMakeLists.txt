cmake_minimum_required(VERSION 2.8.10)
project( HAL )

# Add to module path, so we can find our cmake modules
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )

if(ANDROID)
    set(ANDROID_PACKAGE_NAME "edu.gwu.robotics.hal")
    include(AndroidUtils)
endif()

################################################################################

option(BUILD_SHARED_LIBS "Build shared libraries." ON)

add_subdirectory( PbMsgs )
set(PbMsgs_DIR ${PbMsgs_BINARY_DIR})

option(BUILD_NODE OFF "Build Node library")
if (BUILD_NODE)
  add_subdirectory(Node)
  set(NODE_DIR ${NODE_BINARY_DIR})
endif()

add_subdirectory( HAL )
set(HAL_DIR ${HAL_BINARY_DIR})


set( BUILD_APPLICATIONS true CACHE BOOL force )
if( BUILD_APPLICATIONS )
    add_subdirectory( Applications )
endif()
