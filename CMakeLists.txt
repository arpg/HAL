cmake_minimum_required(VERSION 2.8)

project( RPG )


add_subdirectory( PbMsgs ) # the BEST LIBRARY
add_subdirectory( HAL ) # HAL
add_subdirectory( Utils )

set( Build_LogTools false CACHE BOOL force )
if( Build_LogTools )
    add_subdirectory( LogTools ) # the "BEAVER"
endif()
