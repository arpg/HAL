cmake_minimum_required(VERSION 2.8)
project( HAL )
set(VERSION_MAJOR 0)
set(VERSION_MINOR 1)
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR})

string( TOLOWER ${PROJECT_NAME} LIBRARY_NAME )

# Add to module path, so we can find our cmake modules
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Wextra")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

#############################################################################
# Find required libraries

find_package( OpenCV2 REQUIRED )
find_package( PbMsgs 0.1 REQUIRED )
find_package( Sophus REQUIRED )

include_directories(${Sophus_INCLUDE_DIR})


set_property( GLOBAL PROPERTY P_INCLUDE_DIRS
    ${PbMsgs_INCLUDE_DIRS} ${OpenCV2_INCLUDE_DIRS}
)
set_property( GLOBAL PROPERTY P_LIBRARIES
    ${PbMsgs_LIBRARIES} ${OpenCV2_LIBRARIES}
)

#############################################################################
# HAL macros for driver writers.
set( HAL_DIR ${CMAKE_CURRENT_SOURCE_DIR} )

macro( add_to_hal_include_dirs )
    foreach( dir ${ARGN} )
        set_property( GLOBAL APPEND PROPERTY P_INCLUDE_DIRS "${dir}" )
    endforeach()
endmacro()

macro( add_to_hal_libraries )
    foreach( lib ${ARGN} )
        get_target_property( libpath ${lib} LOCATION )
        if(libpath)
            set_property( GLOBAL APPEND PROPERTY P_LIBRARIES "${libpath}" )
        else()
            set_property( GLOBAL APPEND PROPERTY P_LIBRARIES "${lib}" )
        endif()
    endforeach()
endmacro()

macro( add_to_hal_sources )
    file(RELATIVE_PATH _relPath "${HAL_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
    foreach(_src ${ARGN})
        if(_relPath)
            set_property( GLOBAL APPEND PROPERTY P_SOURCES "${_relPath}/${_src}" )
        else()
            set_property( GLOBAL APPEND PROPERTY P_SOURCES "${_src}" )
        endif()
    endforeach()
endmacro()

macro( hal_set_compile_flags file flags )
    set_property( GLOBAL APPEND PROPERTY COMPILER_OPTS_SOURCES "${file}" )
    set_property( GLOBAL APPEND PROPERTY COMPILER_OPTS_FLAGS "${flags}" )
endmacro()

#############################################################################
# Add Devices

add_subdirectory( Devices )
add_subdirectory( Camera )
add_subdirectory( IMU )
add_subdirectory( Posys )
add_subdirectory( Encoder )
#add_subdirectory( Car )
add_subdirectory( Utils )

#############################################################################
# Setup libraries

set(LIB_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_CURRENT_BINARY_DIR}/.. )
get_property( INTERNAL_INC  GLOBAL PROPERTY P_INCLUDE_DIRS)
get_property( INTERNAL_LIBS GLOBAL PROPERTY P_LIBRARIES)
get_property( HAL_SOURCES GLOBAL PROPERTY P_SOURCES)

# this is a horrible hack in order to set compiler flag properties to individual files
get_property( C_O_S GLOBAL PROPERTY COMPILER_OPTS_SOURCES)
get_property( C_O_F GLOBAL PROPERTY COMPILER_OPTS_FLAGS)

list(LENGTH C_O_S len_c_o_s)
math(EXPR len_c_o_s "${len_c_o_s} - 1")

foreach(val RANGE ${len_c_o_s})
  list(GET C_O_S ${val} source)
  list(GET C_O_F ${val} flag)
    set_source_files_properties( ${source} PROPERTIES COMPILE_FLAGS ${flag} )
endforeach()


if( ANDROID )
    list(APPEND INTERNAL_LIBS "z")
endif()

include_directories( ${LIB_INC_DIR} )
include_directories( ${INTERNAL_INC} )
add_library(hal ${HAL_SOURCES})
target_link_libraries( hal ${INTERNAL_LIBS} )

#######################################################
## Create configure file for inclusion in library

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/config.h"
)

#######################################################

# This relative path allows installed files to be relocatable.
set( CMAKECONFIG_INSTALL_DIR "lib/cmake/${PROJECT_NAME}" )
file( RELATIVE_PATH REL_INCLUDE_DIR
    "${CMAKE_INSTALL_PREFIX}/${CMAKECONFIG_INSTALL_DIR}"
    "${CMAKE_INSTALL_PREFIX}/include" )

# Export library for easy inclusion from other cmake projects.
file( REMOVE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake" )
export( TARGETS ${LIBRARY_NAME}
        APPEND FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake" )

# Version information
configure_file("${PROJECT_NAME}ConfigVersion.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY)

# Build tree config
set( EXPORT_LIB_INC_DIR "${LIB_INC_DIR}" )
CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY IMMEDIATE )

# Install tree config
set( EXPORT_LIB_INC_DIR "\${${PROJECT_NAME}_CMAKE_DIR}/${REL_INCLUDE_DIR}" )
configure_file( "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY )

# Add package to CMake package registery for use from the build tree
export( PACKAGE ${PROJECT_NAME} )

#######################################################
## Install headers / targets

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config.h"
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${LIBRARY_NAME}
)
install(TARGETS ${LIBRARY_NAME}
  EXPORT "${PROJECT_NAME}Targets"
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

#######################################################
## Install CMake config

INSTALL(
    FILES "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
          "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION ${CMAKECONFIG_INSTALL_DIR} )

install( EXPORT "${PROJECT_NAME}Targets" DESTINATION ${CMAKECONFIG_INSTALL_DIR} )
