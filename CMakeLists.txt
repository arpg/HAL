cmake_minimum_required(VERSION 2.8)

project( RPG )

# Add generic libs/headers we often use
include( Common.cmake )

# RPG macros for driver writers.
macro( add_to_rpg_include_dirs dirs )
    list( APPEND RPG_INCLUDE_DIRS ${dirs} )
    set( RPG_INCLUDE_DIRS ${RPG_INCLUDE_DIRS} CACHE INTERNAL "" FORCE )
endmacro()

macro( add_to_rpg_libraries libs )
    foreach( lib in ${libs} )
        get_target_property( _LIBRARY ${lib} LOCATION )
#        message( STATUS "looking for ${lib} got ${_LIBRARY}" )
        if( ${_LIBRARY} STREQUAL "_LIBRARY-NOTFOUND" )
#            message( STATUS "FUCK FUCK FUCK" )
        else()
            list( APPEND RPG_LIBRARIES ${_LIBRARY} )
            set( RPG_LIBRARIES ${RPG_LIBRARIES} CACHE INTERNAL "" FORCE )
        endif()
    endforeach()
endmacro()

macro( add_to_rpg_link_directireis dirs )
    list( APPEND RPG_LINK_DIRECTORIES ${dirs} )
    set( RPG_LINK_DIRECTORIES ${RPG_LINK_DIRECTORIES} CACHE INTERNAL "" FORCE )
endmacro()


# Add to module path, so we can find our cmake modules
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )

# look for opencv
#find_package( OpenCV REQUIRED )
#include_directories( ${OpenCV_INCLUDE_DIRS} )
#link_libraries( ${OpenCV_LIBRARIES} )
#link_directories( ${OpenCV_LINK_DIRECTORIES} )

# find MVL
find_package( MVL REQUIRED )
include_directories( ${MVL_INCLUDE_DIRS} )
link_libraries( ${MVL_LIBRARIES} )
link_directories( ${MVL_LIBRARY_DIRS} )

include_directories( ${CMAKE_SOURCE_DIR}/.. )


set( RPG_INCLUDE_DIRS ${MVL_INCLUDE_DIRS} ${COMMON_INCLUDE_DIRS} CACHE INTERNAL "" FORCE )
set( RPG_LIBRARIES ${MVL_LIBRARIES} ${COMMON_LIBRARIES} CACHE INTERNAL "" FORCE )


add_subdirectory( Devices )


# Create the RPGConfig.cmake file for the build tree.
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/RPGConfig.cmake.in
            ${CMAKE_CURRENT_BINARY_DIR}/RPGConfig.cmake @ONLY IMMEDIATE )

# Add module to CMake package registery.
export( PACKAGE RPG )
