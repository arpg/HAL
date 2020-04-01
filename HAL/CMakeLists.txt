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

if(NOT MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-inconsistent-missing-override -Wextra")
endif()

#############################################################################
# Find required libraries
find_package( Eigen3 REQUIRED )
find_package( protobuf REQUIRED CONFIG)
find_package( Sophus REQUIRED )
find_package( gflags REQUIRED CONFIG)
find_package( glog REQUIRED CONFIG)
find_package( tinyxml2 QUIET CONFIG)

if(tinyxml2_FOUND)
  add_definitions(-DHAVE_TINYXML2)
  list(APPEND LINK_LIBS  tinyxml2)
endif()


find_package(OpenCV QUIET COMPONENTS core)
if(NOT OpenCV_FOUND)
  message(WARNING "No OpenCV found; camera drivers disabled.")
endif()
set(OpenCV_FOUND FALSE)
if(OpenCV_VERSION_MAJOR EQUAL 2)
    find_package(OpenCV QUIET COMPONENTS core imgproc highgui)
elseif(OpenCV_VERSION_MAJOR EQUAL 3)
    find_package(OpenCV QUIET COMPONENTS core imgproc imgcodecs videoio)
endif()

if(WIN32)
    find_path(DIRENT_INCLUDE_DIR NAMES dirent.h)
    find_package(dlfcn-win32 REQUIRED CONFIG)
    list( APPEND LINK_LIBS  dlfcn-win32::dl)
    find_path(POSIX_shim_INCLUDE_DIR NAMES unistd.h)
endif()

list( APPEND USER_INC   ${EIGEN3_INCLUDE_DIR} )
list( APPEND USER_INC   ${Sophus_INCLUDE_DIRS} )


list( APPEND LINK_LIBS protobuf::libprotobuf glog::glog gflags_shared)

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
            if(NOT WIN32)
              set(prop LOCATION)
            else()
              set(prop IMPORTED_IMPLIB_DEBUG)
              get_target_property(libpath ${lib} ${prop})
              if(NOT libpath)
                set(prop IMPORTED_IMPLIB_RELEASE)
              endif()
            endif()
            get_target_property(libpath ${lib} ${prop})
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
add_subdirectory( IMU )
add_subdirectory( Gamepad )
add_subdirectory( Posys )
add_subdirectory( LIDAR )
add_subdirectory( Car )
add_subdirectory( Utils )

if(OpenCV_FOUND)
add_subdirectory( Camera )
endif()

if(BUILD_Velodyne)
add_subdirectory( LIDAR )
endif()

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
    ${PROTO_DIR}/Gamepad.proto
    ${PROTO_DIR}/Car.proto
    ${PROTO_DIR}/Pose.proto
    ${PROTO_DIR}/Lidar.proto
    ${PROTO_DIR}/NodeCamMessage.proto
    ${PROTO_DIR}/Command.proto
)
IF(WIN32)
    set(PROTOBUF_GENERATE_CPP_EXPORT dllexport_decl=HAL_EXPORT:)
ENDIF()
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

#############################################################################
## Protobuf

find_package(OpenCV QUIET COMPONENTS core)

set(OpenCV_FOUND FALSE)
if(OpenCV_VERSION_MAJOR EQUAL 2)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
elseif(OpenCV_VERSION_MAJOR EQUAL 3)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio highgui)
endif()

list(APPEND HAL_SOURCES
    ${PROTO_DIR}/Logger.cpp
    ${PROTO_DIR}/Reader.cpp
   )

list(APPEND HAL_HEADERS
    ${PROTO_DIR}/Logger.h
    ${PROTO_DIR}/Reader.h
    ${PROTO_DIR}/Matrix.h
    ${PROTO_DIR}/Pose.h
    ${PROTO_DIR}/Command.h
   )

if(OpenCV_FOUND)
list(APPEND HAL_SOURCES
    ${PROTO_DIR}/Image.cpp
   )
list(APPEND HAL_HEADERS
    ${PROTO_DIR}/Image.h
    ${PROTO_DIR}/ImageArray.h
    ${PROTO_DIR}/ImagePyramid.h
   )
endif()

if(BUILD_Velodyne)
    list(APPEND HAL_SOURCES ${PROTO_DIR}/Velodyne.cpp)
    list(APPEND HAL_HEADERS ${PROTO_DIR}/Velodyne.h)
endif()
##########################################################################

include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR} )
if(TinyXML2_FOUND)
  include_directories( ${TinyXML2_INCLUDE_DIR} )
endif()
include_directories( ${INTERNAL_INC} )
include_directories( ${USER_INC} )
include_directories( ${CMAKE_CURRENT_BINARY_DIR}/Messages )
include_directories( ${CMAKE_BINARY_DIR} )
include_directories( ${OpenCV_INCLUDE_DIRS} )
link_directories(/usr/local/lib)
link_directories(${CMAKE_INSTALL_PREFIX}/lib)

set(HAL_LIBS
  ${INTERNAL_LIBS}
  ${LINK_LIBS}
  ${OpenCV_LIBS}
  )

list( REMOVE_ITEM HAL_LIBS "debug" )
list( REMOVE_ITEM HAL_LIBS "optimized" )

set(HAL_INCLUDES
    ${USER_INC} )

add_library( hal ${HAL_SOURCES} )
target_link_libraries( hal ${HAL_LIBS} )

SET_TARGET_PROPERTIES(hal PROPERTIES PREFIX lib)

if (BUILD_SHARED_LIBS)

  if (WIN32)
    set (_EXPORT "__declspec(dllexport)")
    set (_IMPORT "__declspec(dllimport)")
  endif()

  target_compile_definitions (hal PRIVATE
    "HAL_EXPORT=${_EXPORT}")
  target_compile_definitions (hal INTERFACE
    "HAL_EXPORT=${_IMPORT}")

else()
   target_compile_definitions (hal PUBLIC HAL_EXPORT=)
endif()

if(WIN32)
  target_include_directories(hal PUBLIC ${DIRENT_INCLUDE_DIR} )
  target_include_directories(hal PUBLIC ${POSIX_shim_INCLUDE_DIR} )
endif()

if(ANDROID)
    set_target_properties(hal PROPERTIES LINK_FLAGS " -Wl,--build-id ")
endif()


########################################################
## Create configure file for inclusion in library
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/config.h" )

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/Devices/SharedLoad.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/Devices/SharedLoad.h" )

set( GENERATED_HEADERS ${PROTO_HDRS} ${CMAKE_CURRENT_BINARY_DIR}/config.h
    ${CMAKE_CURRENT_BINARY_DIR}/ThirdParty/ThirdPartyConfig.h ${CMAKE_CURRENT_BINARY_DIR}/Devices/SharedLoad.h)

IF(USB1_LIBRARY)
    set(HAL_LIBS ${HAL_LIBS} ${USB1_LIBRARY})
ENDIF()

include (CMakePackageConfigHelpers)


install (TARGETS hal
  EXPORT hal-targets
  RUNTIME DESTINATION bin
  PUBLIC_HEADER DESTINATION include/HAL
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

set(LOAD_DEPENDENCY "find_package( protobuf CONFIG )
find_dependency( gflags )
find_dependency( glog )")

if(tinyxml2_FOUND)
  set(LOAD_DEPENDENCY "${LOAD_DEPENDENCY}
find_dependency(tinyxml2)")
endif()

if(WIN32)
  set(LOAD_DEPENDENCY "${LOAD_DEPENDENCY}
find_dependency(dlfcn-win32)")
endif()


configure_package_config_file (hal-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/hal-config.cmake
  INSTALL_DESTINATION lib/cmake/hal
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file (hal-config-version.cmake VERSION
  ${HAL_VERSION} COMPATIBILITY SameMajorVersion)

export (TARGETS hal NAMESPACE hal:: FILE hal-targets.cmake)
export (PACKAGE HAL)

install (FILES
  ${CMAKE_CURRENT_BINARY_DIR}/hal-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/hal-config-version.cmake
  DESTINATION lib/cmake/hal)

install (EXPORT hal-targets NAMESPACE hal:: DESTINATION lib/cmake/hal)

foreach(hdr IN LISTS HAL_HEADERS )
    get_filename_component( _fp ${hdr} ABSOLUTE )
    file( RELATIVE_PATH _rpath ${CMAKE_SOURCE_DIR} ${_fp} )
    get_filename_component( _dir ${_rpath} DIRECTORY )
    install( FILES ${_fp}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${_dir} )
endforeach()

foreach(hdr IN LISTS GENERATED_HEADERS )
    get_filename_component( _fp ${hdr} ABSOLUTE )
    file( RELATIVE_PATH _rpath ${CMAKE_BINARY_DIR} ${_fp} )
    get_filename_component( _dir ${_rpath} DIRECTORY )
    install( FILES ${_fp}
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${_dir} )
endforeach()

