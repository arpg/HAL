# Version 2.8.10 onwards is required for dependency build to work correctly.
cmake_minimum_required(VERSION 2.8.10)
project( HAL )

# Add to module path, so we can find our cmake modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

if(ANDROID)
    set(ANDROID_PACKAGE_NAME "edu.colorado.arpg")
endif()

################################################################################

option( BUILD_SHARED_LIBS "Build shared libraries." ON )

include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
add_subdirectory( HAL )

set( HAL_DIR ${CMAKE_CURRENT_BINARY_DIR} )
set( HAL_LIBRARIES ${HAL_LIBS} hal )
set( HAL_INCLUDE_DIRS ${HAL_INCLUDES} )
link_libraries( ${HAL_LIBS} hal )

include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR} )

# make an uninstall target
include(${CMAKE_MODULE_PATH}/cmake_uninstall.cmake.in)
add_custom_target(uninstall
    COMMAND "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
