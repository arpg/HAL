cmake_minimum_required(VERSION 2.8)
project( RPG )

option(BUILD_SHARED_LIBS "Build shared libraries." ON)

add_subdirectory( PbMsgs )
add_subdirectory( HAL )


set( BUILD_APPLICATIONS true CACHE BOOL force )
if( BUILD_APPLICATIONS AND NOT ANDROID )
    add_subdirectory( Applications )
endif()
