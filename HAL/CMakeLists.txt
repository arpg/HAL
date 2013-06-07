cmake_minimum_required(VERSION 2.8)
project( HAL )
set(VERSION_MAJOR 0)
set(VERSION_MINOR 1)
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR})

string( TOLOWER ${PROJECT_NAME} LIBRARY_NAME )

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
    ${PbMsgs_INCLUDE_DIRS} ${OpenCV2_INCLUDE_DIRS}
)
set_property( GLOBAL PROPERTY P_LIBRARIES
    ${PbMsgs_LIBRARIES} ${OpenCV2_LIBRARIES}
)

#############################################################################
# HAL macros for driver writers.
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

set(LIB_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_CURRENT_BINARY_DIR}/.. )
get_property( INTERNAL_INC  GLOBAL PROPERTY P_INCLUDE_DIRS)
get_property( INTERNAL_LIBS GLOBAL PROPERTY P_LIBRARIES)
get_property( HAL_SOURCES GLOBAL PROPERTY P_SOURCES)

if( ANDROID )
    # TODO: Figure out how to set these from sub-dirs.
    set_source_files_properties( Camera/Drivers/Android/CameraListener.cpp PROPERTIES COMPILE_FLAGS "-fno-rtti -fpermissive" )
    set_source_files_properties( Camera/Drivers/Android/AndroidDriver.cpp PROPERTIES COMPILE_FLAGS "-fpermissive" )
    set_source_files_properties( Camera/Drivers/Android/AndroidFactory.cpp PROPERTIES COMPILE_FLAGS "-fpermissive" )
    list(APPEND INTERNAL_LIBS "zlib")
endif()

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
export( TARGETS ${LIBRARY_NAME} pbmsgs
        FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake" )

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

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/include/${LIBRARY_NAME}/config.h"
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${LIBRARY_NAME}
)
#install(DIRECTORY ${INCDIR}
#  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
#)
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
