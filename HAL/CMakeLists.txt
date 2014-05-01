cmake_minimum_required(VERSION 2.8)
project( HAL )
set(VERSION_MAJOR 0)
set(VERSION_MINOR 1)
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR})

include(def_library)
include(export_package)

string( TOLOWER ${PROJECT_NAME} LIBRARY_NAME )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Wextra")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

#############################################################################
# Find required libraries

find_package( OpenCV2 REQUIRED )
find_package( PbMsgs 0.1 REQUIRED )
find_package( Sophus REQUIRED )
find_package( Boost COMPONENTS system filesystem REQUIRED )
find_package( TinyXML2 )

if(TinyXML2_FOUND)
  add_definitions(-DHAVE_TINYXML2)
endif()

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


if(TinyXML2_FOUND)
  include_directories( ${TinyXML2_INCLUDE_DIR} )
endif()

include_directories( ${LIB_INC_DIR} )
include_directories( ${INTERNAL_INC} )

def_library(hal
  SOURCES ${HAL_SOURCES}
  LINK_LIBS ${INTERNAL_LIBS} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY}
  PACKAGE HAL)

if(TinyXML2_FOUND)
  target_link_libraries( hal ${TinyXML2_LIBRARIES} )
endif()

export_package(HAL
  TARGETS hal
  VERSION ${VERSION}
  LIBRARY hal
  LIBRARIES hal
  INCLUDE_DIRS ${LIB_INC_DIR} ${USER_INC}
  )

#######################################################
## Create configure file for inclusion in library

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/config.h"
  )

#######################################################
#######################################################
## Install headers / targets
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config.h"
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${LIBRARY_NAME}
)