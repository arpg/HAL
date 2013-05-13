cmake_minimum_required(VERSION 2.8)

project( RPG )

option(BUILD_SHARED_LIBS "Build shared libraries." ON)

add_subdirectory( PbMsgs )
add_subdirectory( HAL )

#add_subdirectory( Applications )

set( Build_LogTools false CACHE BOOL force )
if( Build_LogTools )
    add_subdirectory( LogTools )
endif()
