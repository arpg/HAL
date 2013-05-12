cmake_minimum_required(VERSION 2.8)

project( RPG )

add_subdirectory( PbMsgs )
add_subdirectory( HAL )

set( Build_LogTools false CACHE BOOL force )
if( Build_LogTools )
    add_subdirectory( LogTools )
endif()
