cmake_minimum_required(VERSION 2.8)
project( HAL )
set(HAL_VERSION_MAJOR 0)
set(HAL_VERSION_MINOR 2)
set(HAL_VERSION ${HAL_VERSION_MAJOR}.${HAL_VERSION_MINOR})
set( Messages_VERSION 2 )

# Policy CMP0023 allows to mix old and new interfaces of target_link_libraries
cmake_policy(SET CMP0023 OLD)
cmake_policy(SET CMP0022 OLD)

if(POLICY CMP0042)
    cmake_policy(SET CMP0042 NEW) # MACOSX_RPATH
endif()

include( install_package )

string(TOLOWER ${PROJECT_NAME} LIBRARY_NAME)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

#############################################################################
# Find required libraries
find_package( Eigen3 REQUIRED )
find_package( Protobuf REQUIRED )
if(ANDROID)
    # Override to use hosts protoc compiler
    unset(PROTOBUF_PROTOC_EXECUTABLE CACHE)
    find_host_package(Protobuf REQUIRED)
endif()
find_package( Sophus REQUIRED )
find_package( GLog REQUIRED )
find_package( TinyXML2 QUIET )

if(TinyXML2_FOUND)
  add_definitions(-DHAVE_TINYXML2)
  list(APPEND LINK_LIBS  ${TinyXML2_LIBRARIES})
  list(APPEND USER_INC   ${TinyXML2_INCLUDE_DIRS})
endif()

list( APPEND USER_INC   ${EIGEN3_INCLUDE_DIR} )
list( APPEND USER_INC   ${Sophus_INCLUDE_DIRS} )
list( APPEND USER_INC   ${PROTOBUF_INCLUDE_DIR} )
list( APPEND USER_INC   ${GLog_INCLUDE_DIRS} )

include_directories( ${LIB_INC_DIR} ${USER_INC} )

list( APPEND LINK_LIBS ${PROTOBUF_LIBRARIES} ${GLog_LIBRARIES})

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
    if( NOT child STREQUAL "CMakeFiles" )
      if(IS_DIRECTORY ${curdir}/${child})
          set(dirlist ${dirlist} ${child})
      endif()
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

set( PROTO_DIR Messages )

set( _PROTO_SRCS
    # aux
    ${PROTO_DIR}/Matrix.proto

    # header files
    ${PROTO_DIR}/Header.proto
    ${PROTO_DIR}/CameraModel.proto

    # message files
    ${PROTO_DIR}/Messages.proto
    ${PROTO_DIR}/Camera.proto
    ${PROTO_DIR}/Image.proto
    ${PROTO_DIR}/Imu.proto
    ${PROTO_DIR}/Pose.proto
    ${PROTO_DIR}/Encoder.proto
    ${PROTO_DIR}/Lidar.proto
    ${PROTO_DIR}/NodeCamMessage.proto
    ${PROTO_DIR}/NodeCar.proto
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${_PROTO_SRCS})

add_to_hal_sources( ${PROTO_SRCS} )

#############################################################################
# Setup libraries

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

#############################################################################
## Protobuf

list(APPEND HAL_SOURCES
    ${PROTO_DIR}/Logger.cpp
    ${PROTO_DIR}/Reader.cpp
    ${PROTO_DIR}/Image.cpp
   )

list(APPEND HAL_HEADERS
    ${PROTO_DIR}/Logger.h
    ${PROTO_DIR}/Reader.h
    ${PROTO_DIR}/Image.h
    ${PROTO_DIR}/ImageArray.h
    ${PROTO_DIR}/ImagePyramid.h 
    ${PROTO_DIR}/Velodyne.h
    ${PROTO_DIR}/Matrix.h
    ${PROTO_DIR}/Pose.h
   )

if(BUILD_Velodyne)
    list(APPEND HAL_SOURCES ${PROTO_DIR}/Velodyne.cpp)
    list(APPEND HAL_HEADERS ${PROTO_DIR}/Velodyne.h)
endif()

##########################################################################

include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR} )
include_directories( ${LIB_INC_DIR} )
include_directories( ${INTERNAL_INC} )
include_directories( ${CMAKE_CURRENT_BINARY_DIR}/Messages )

set(HAL_LIBS
  ${INTERNAL_LIBS}
  ${PROTOBUF_LIBRARIES}
  ${LINK_LIBS}
  CACHE STRING "HAL required libraries"
  )

list( REMOVE_ITEM HAL_LIBS "debug" )
list( REMOVE_ITEM HAL_LIBS "optimized" )

set(HAL_INCLUDES
    ${LIB_INC_DIR}
    ${USER_INC}
    CACHE STRING "HAL required includes" )


add_library( hal ${HAL_SOURCES} )
target_link_libraries( hal ${HAL_LIBS} )

########################################################
## Create configure file for inclusion in library
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/config.h" )

set( GENERATED_HEADERS ${PROTO_HDRS} ${CMAKE_CURRENT_BINARY_DIR}/config.h
    ${CMAKE_CURRENT_BINARY_DIR}/ThirdParty/ThirdPartyConfig.h )
    
install_package(
    PKG_NAME HAL
    LIB_NAME hal
    VERSION ${HAL_VERSION}
    INSTALL_HEADERS ${HAL_HEADERS}
    INSTALL_GENERATED_HEADERS ${GENERATED_HEADERS}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include
    INCLUDE_DIRS ${HAL_INCLUDES}
    LINK_LIBS ${HAL_LIBS}
    )

include_directories( ${CMAKE_BINARY_DIR} )


