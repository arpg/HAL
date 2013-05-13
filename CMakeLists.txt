cmake_minimum_required(VERSION 2.8)

project( RPG )

option(BUILD_SHARED_LIBS "Build shared libraries." ON)

add_subdirectory( PbMsgs )
add_subdirectory( HAL )


set( Build_Applications true CACHE BOOL force )
if( Build_Applications )
    add_subdirectory( Applications )
endif()
