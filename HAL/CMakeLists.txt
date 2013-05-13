cmake_minimum_required(VERSION 2.8)

# Add to module path, so we can find our cmake modules
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

#############################################################################
# Find required libraries

find_package( OpenCV2 REQUIRED )
find_package( PbMsgs REQUIRED )

set_property( GLOBAL PROPERTY P_INCLUDE_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_CURRENT_BINARY_DIR}/..
    ${PbMsgs_INCLUDE_DIRS} ${OpenCV2_INCLUDE_DIRS} 
)
set_property( GLOBAL PROPERTY P_LIBRARIES 
    ${PbMsgs_LIBRARIES} ${OpenCV2_LIBRARIES}
)

#############################################################################
# RPG macros for driver writers.
macro( add_to_hal_include_dirs )
    foreach( dir ${ARGN} )
        set_property( GLOBAL APPEND PROPERTY P_INCLUDE_DIRS "${dir}" )
    endforeach()
endmacro()

macro( add_to_hal_libraries )
    foreach( lib ${ARGN} )
        get_target_property( libpath ${lib} LOCATION )
        set_property( GLOBAL APPEND PROPERTY P_LIBRARIES "${libpath}" )
    endforeach()
endmacro()

macro( add_to_hal_sources )
    file(RELATIVE_PATH _relPath "${CMAKE_SOURCE_DIR}/HAL" "${CMAKE_CURRENT_SOURCE_DIR}")
    foreach(_src ${ARGN})
        if(_relPath)
            set_property( GLOBAL APPEND PROPERTY P_SOURCES "${_relPath}/${_src}" )
        else()
            set_property( GLOBAL APPEND PROPERTY P_SOURCES "${_src}" )
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
# Setup libraries

get_property( HAL_INCLUDE_DIRS GLOBAL PROPERTY P_INCLUDE_DIRS)
get_property( HAL_LIBRARIES GLOBAL PROPERTY P_LIBRARIES)
get_property( HAL_SOURCES GLOBAL PROPERTY P_SOURCES)

include_directories( ${HAL_INCLUDE_DIRS} )
add_library(hal ${HAL_SOURCES})
target_link_libraries( hal ${HAL_LIBRARIES} )

#############################################################################
# Export library

add_to_hal_libraries(hal)
get_property( HAL_LIBRARIES GLOBAL PROPERTY P_LIBRARIES)

# Create the HALConfig.cmake file for the build tree.
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/HALConfig.cmake.in
            ${CMAKE_CURRENT_BINARY_DIR}/HALConfig.cmake @ONLY IMMEDIATE )

# Add module to CMake package registery.
export( PACKAGE HAL )
