cmake_minimum_required(VERSION 2.8)


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
ENDIF()


# RPG macros for driver writers.
macro( add_to_hal_include_dirs dirs )
    list( APPEND HAL_INCLUDE_DIRS ${dirs} )
    set( HAL_INCLUDE_DIRS ${HAL_INCLUDE_DIRS} CACHE INTERNAL "" FORCE )
endmacro()

macro( add_to_hal_libraries libs )
    foreach( lib ${libs} )
        get_target_property( _LIBRARY ${lib} LOCATION )
        if( ${_LIBRARY} STREQUAL "_LIBRARY-NOTFOUND" )
        else()
            message( STATUS "******* adding '${_LIBRARY}' ")
            list( INSERT HAL_LIBRARIES 0 ${_LIBRARY} )
            set( HAL_LIBRARIES ${HAL_LIBRARIES} CACHE INTERNAL "" FORCE )
        endif()
    endforeach()
endmacro()

macro( add_to_hal_link_directories dirs )
  list( APPEND HAL_LINK_DIRECTORIES ${dirs} )
  set( HAL_LINK_DIRECTORIES ${HAL_LINK_DIRECTORIES} CACHE INTERNAL "" FORCE )
endmacro()


# Add to module path, so we can find our cmake modules
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )

# look for opencv
find_package( OpenCV2 REQUIRED )
include_directories( ${OpenCV2_INCLUDE_DIRS} )
link_libraries( ${OpenCV2_LIBRARIES} )

# boost
find_package( Boost COMPONENTS thread REQUIRED )

# find MVL
find_package( MVL REQUIRED )
include_directories( ${MVL_INCLUDE_DIRS} )
link_libraries( ${MVL_LIBRARIES} )
#link_directories( ${MVL_LIBRARY_DIRS} )


# find PbMsgs
find_package( PbMsgs REQUIRED )
include_directories( ${PBMSGS_INCLUDE_DIRS} )
link_libraries( ${PBMSGS_LIBRARIES} )


include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/.. )


set( HAL_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${PBMSGS_INCLUDE_DIRS} ${MVL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV2_INCLUDE_DIRS} CACHE INTERNAL "" FORCE )
set( HAL_LIBRARIES ${PBMSGS_LIBRARIES} ${MVL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV2_LIBRARIES} CACHE INTERNAL "" FORCE )


add_subdirectory( Camera )
add_subdirectory( IMU )
#add_subdirectory( Car )


# Code to cleanup apple frameworks to "export" properly.
# This is needed because spaces in the list of LIBRARIES in the .cmake we are
# creating here MUST BE ESCAPPED.  Otherwise, when an external project imports
# the list, CMAKE turns the space into a semicolon, resulting in mistakes
# like "-lframework;-lIOKit" instead of "-framework IOKit".
set( _HAL_LIBRARIES "" )
foreach( lib ${HAL_LIBRARIES})
    string( REPLACE "framework " "framework\\ " lib ${lib} )
    list( APPEND _HAL_LIBRARIES ${lib} )
endforeach()
set( HAL_LIBRARIES ${_HAL_LIBRARIES} CACHE INTERNAL "" FORCE )

# Create the HALConfig.cmake file for the build tree.
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/HALConfig.cmake.in
            ${CMAKE_CURRENT_BINARY_DIR}/HALConfig.cmake @ONLY IMMEDIATE )


# Add module to CMake package registery.
export( PACKAGE HAL )
