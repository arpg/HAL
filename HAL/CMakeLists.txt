cmake_minimum_required(VERSION 2.8)
project( HAL )
set(HAL_VERSION_MAJOR 0)
set(HAL_VERSION_MINOR 2)
set(HAL_VERSION ${HAL_VERSION_MAJOR}.${HAL_VERSION_MINOR})

# Policy CMP0023 allows to mix old and new interfaces of target_link_libraries
cmake_policy(SET CMP0023 OLD)
cmake_policy(SET CMP0022 OLD)

include( install_package )

string(TOLOWER ${PROJECT_NAME} LIBRARY_NAME)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

#############################################################################
# Find required libraries
find_package( Eigen3 REQUIRED )
find_package( Protobuf REQUIRED )
find_package( Sophus REQUIRED )
find_package( MINIGLOG REQUIRED )
find_package( TinyXML2 QUIET )
find_package( OpenCV2 QUIET )
if( OpenCV2_FOUND )
    add_definitions( -DHAVE_OPENCV )
    list(APPEND LINK_LIBS ${OpenCV2_LIBRARIES})
    list(APPEND USER_INC ${OpenCV2_INCLUDE_DIRS})
else()
    message(STATUS "OpenCV2 not found")
endif()
if(TinyXML2_FOUND)
  add_definitions(-DHAVE_TINYXML2)
  list(APPEND LINK_LIBS  ${TinyXML2_LIBRARIES})
  list(APPEND USER_INC   ${TinyXML2_INCLUDE_DIRS})
endif()

list( APPEND USER_INC   ${CMAKE_CURRENT_BINARY_DIR} )
list( APPEND USER_INC   ${CMAKE_CURRENT_SOURCE_DIR} )
list( APPEND USER_INC   ${EIGEN3_INCLUDE_DIR} )
list( APPEND USER_INC   ${Sophus_INCLUDE_DIR} )
list( APPEND USER_INC   ${PROTOBUF_INCLUDE_DIR} )
list( APPEND USER_INC   ${MINIGLOG_INCLUDE_DIRS} )

list( APPEND LIB_INC_DIR "${CMAKE_CURRENT_BINARY_DIR}/..;${CMAKE_CURRENT_SOURCE_DIR}/.." )

include_directories( ${LIB_INC_DIR} ${USER_INC} )

list( APPEND LINK_LIBS ${PROTOBUF_LIBRARIES} ${MINIGLOG_LIBRARIES})

############################################################################
# Build messages library
add_subdirectory( Messages )
############################################################################


# Collect all driver includes and libraries in P_INCLUDE_DIRS and P_LIBRARIES
set_property( GLOBAL PROPERTY P_INCLUDE_DIRS ${OpenCV2_INCLUDE_DIRS} )
set_property( GLOBAL PROPERTY P_LIBRARIES ${OpenCV2_LIBRARIES} )

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
      # Process targets correctly
      if (TARGET ${lib})

  # If the library is NOT imported, ie is in this project, we
  # want to depend on it directly rather than through its path
        get_target_property(is_lib_imported ${lib} IMPORTED)
  if (NOT ${is_lib_imported})
    set_property( GLOBAL APPEND PROPERTY P_LIBRARIES "${lib}" )
  else()
    # For imported targets, we just want to depend on the library directly
          get_target_property(libpath ${lib} LOCATION)
    if (libpath)
            set_property( GLOBAL APPEND PROPERTY P_LIBRARIES "${libpath}" )

      # This shouldn't really happen, but let's cover our bases.
    else()
      set_property( GLOBAL APPEND PROPERTY P_LIBRARIES "${lib}" )
    endif()
  endif()
      else()			# Just add the direct path/flag to the list
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

macro( add_to_hal_headers )
    file(RELATIVE_PATH _relPath "${HAL_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
    foreach(_hdr ${ARGN})
        if(_relPath)
            set_property( GLOBAL APPEND PROPERTY P_HEADERS "${_relPath}/${_hdr}" )
        else()
            set_property( GLOBAL APPEND PROPERTY P_HEADERS "${_hdr}" )
        endif()
    endforeach()
endmacro()


macro( hal_set_compile_flags file flags )
    set_property( GLOBAL APPEND PROPERTY COMPILER_OPTS_SOURCES "${file}" )
    set_property( GLOBAL APPEND PROPERTY COMPILER_OPTS_FLAGS "${flags}" )
endmacro()


macro(subdirlist result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
        set(dirlist ${dirlist} ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

#############################################################################
# Add Devices

add_subdirectory( Devices )
add_subdirectory( ThirdParty )
add_subdirectory( Camera )
add_subdirectory( IMU )
add_subdirectory( Posys )
add_subdirectory( Encoder )
add_subdirectory( LIDAR )
add_subdirectory( Car )
add_subdirectory( Utils )

#############################################################################
# Setup libraries

set(LIB_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_CURRENT_BINARY_DIR}/.. )
get_property( INTERNAL_INC  GLOBAL PROPERTY P_INCLUDE_DIRS )
get_property( INTERNAL_LIBS GLOBAL PROPERTY P_LIBRARIES )
get_property( HAL_SOURCES GLOBAL PROPERTY P_SOURCES )
get_property( HAL_HEADERS GLOBAL PROPERTY P_HEADERS )

# this is a horrible hack in order to set compiler flag properties to individual files
get_property( C_O_S GLOBAL PROPERTY COMPILER_OPTS_SOURCES )
get_property( C_O_F GLOBAL PROPERTY COMPILER_OPTS_FLAGS )

list(LENGTH C_O_S len_c_o_s )
math(EXPR len_c_o_s "${len_c_o_s} - 1" )

foreach(val RANGE ${len_c_o_s} )
  list(GET C_O_S ${val} source )
  list(GET C_O_F ${val} flag )
  set_source_files_properties( ${source} PROPERTIES COMPILE_FLAGS ${flag} )
endforeach()

if(TinyXML2_FOUND)
  include_directories( ${TinyXML2_INCLUDE_DIR} )
endif()

include_directories( ${LIB_INC_DIR} )
include_directories( ${INTERNAL_INC} )

set(HAL_LIBS
  ${INTERNAL_LIBS}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${PROTOBUF_LIBRARIES}
  ${LINK_LIBS}
  messages
  CACHE STRING "HAL required libraries"
  )

set(HAL_INCLUDES
    ${Messages_BINARY_DIR}
    ${LIB_INC_DIR}
    ${USER_INC}
    CACHE STRING "HAL required includes" )

add_library( hal ${HAL_SOURCES} ${HAL_HEADERS} )
target_link_libraries( hal ${HAL_LIBS} )

install_package(
    PKG_NAME HAL
    LIB_NAME hal
    VERSION ${HAL_VERSION}
    INSTALL_HEADERS ${HAL_HEADERS}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/HAL
    INCLUDE_DIRS ${HAL_INCLUDES}
    LINK_LIBS ${HAL_LIBS}
    )

########################################################
## Create configure file for inclusion in library

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/config.h"
  )

# make an uninstall target
include(${CMAKE_MODULE_PATH}/cmake_uninstall.cmake.in)
add_custom_target(uninstall
    COMMAND "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/HAL/cmake_uninstall.cmake")
