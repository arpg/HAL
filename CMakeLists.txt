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
add_subdirectory( HAL )


set( BUILD_APPLICATIONS true CACHE BOOL force )
if( BUILD_APPLICATIONS )
    add_subdirectory( Applications )
endif()
