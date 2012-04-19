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
#    message( STATUS "DEALING with ${libs}" )
    foreach( lib ${libs} )
#        message( STATUS "LIB: ${lib}" )
        get_target_property( _LIBRARY ${lib} LOCATION )
        if( ${_LIBRARY} STREQUAL "_LIBRARY-NOTFOUND" )
#        message( STATUS "LOCATION: ${_LIBRARY}" )
        else()
#            message( STATUS "looking for ${lib} got ${_LIBRARY}" )
            message( STATUS "******* adding '${_LIBRARY}' ")
            list( INSERT RPG_LIBRARIES 0 ${_LIBRARY} )
            set( RPG_LIBRARIES ${RPG_LIBRARIES} CACHE INTERNAL "" FORCE )
#            message( STATUS "******* result '${RPG_LIBRARIES}' ")
        endif()
    endforeach()
endmacro()

macro( add_to_rpg_link_directories dirs )
  list( APPEND RPG_LINK_DIRECTORIES ${dirs} )
  set( RPG_LINK_DIRECTORIES ${RPG_LINK_DIRECTORIES} CACHE INTERNAL "" FORCE )
endmacro()


# Add to module path, so we can find our cmake modules
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )

# look for opencv
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
link_libraries( ${OpenCV_LIBS} )
link_directories( ${OpenCV_LINK_DIRECTORIES} )

# find MVL
find_package( MVL REQUIRED )
include_directories( ${MVL_INCLUDE_DIRS} )
link_libraries( ${MVL_LIBRARIES} )
link_directories( ${MVL_LIBRARY_DIRS} )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/.. )


set( RPG_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/.. ${MVL_INCLUDE_DIRS} ${COMMON_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} CACHE INTERNAL "" FORCE )
set( RPG_LIBRARIES ${MVL_LIBRARIES} ${COMMON_LIBRARIES} ${OpenCV_LIBS} CACHE INTERNAL "" FORCE )


add_subdirectory( Devices )
add_subdirectory( Robots )
add_subdirectory( Utils )

#message( STATUS "Got RPG_LIBRARIES ${RPG_LIBRARIES}" )

# Code to cleanup apple frameworks to "export" properly.
# This is needed because spaces in the list of LIBRARIES in the .cmake we are
# creating here MUST BE ESCAPPED.  Otherwise, when an external project imports
# the list, CMAKE turns the space into a semicolon, resulting in mistakes
# like "-lframework;-lIOKit" instead of "-framework IOKit".
set( _RPG_LIBRARIES "" )
foreach( lib ${RPG_LIBRARIES})
    string( REPLACE "framework " "framework\\ " lib ${lib} )
    list( APPEND _RPG_LIBRARIES ${lib} )
endforeach()
set( RPG_LIBRARIES ${_RPG_LIBRARIES} CACHE INTERNAL "" FORCE )

# Create the RPGConfig.cmake file for the build tree.
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/RPGConfig.cmake.in
            ${CMAKE_CURRENT_BINARY_DIR}/RPGConfig.cmake @ONLY IMMEDIATE )

# Add module to CMake package registery.
export( PACKAGE RPG )
