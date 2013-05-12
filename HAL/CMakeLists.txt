cmake_minimum_required(VERSION 2.8)

# Add to module path, so we can find our cmake modules
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
ENDIF()

#############################################################################
# Find required libraries

set( HAL_INCLUDE_DIRS "" CACHE INTERNAL "" FORCE)
set( HAL_LIBRARIES "" CACHE INTERNAL "" FORCE)

find_package( OpenCV2 REQUIRED )
find_package( Boost COMPONENTS thread REQUIRED )
find_package( PbMsgs REQUIRED )

list( APPEND HAL_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_CURRENT_BINARY_DIR}/..
    ${PBMSGS_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
    ${OpenCV2_INCLUDE_DIRS} 
)
list( APPEND HAL_LIBRARIES
    ${PBMSGS_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV2_LIBRARIES}
)

include_directories( ${HAL_INCLUDE_DIRS} )
link_libraries( ${HAL_LIBRARIES} )

#############################################################################
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

#############################################################################
# Add Devices

add_subdirectory( Devices )
add_subdirectory( Camera )
#add_subdirectory( IMU )
#add_subdirectory( Car )

#############################################################################
# Export library

# Create the HALConfig.cmake file for the build tree.
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/HALConfig.cmake.in
            ${CMAKE_CURRENT_BINARY_DIR}/HALConfig.cmake @ONLY IMMEDIATE )

# Add module to CMake package registery.
export( PACKAGE HAL )
