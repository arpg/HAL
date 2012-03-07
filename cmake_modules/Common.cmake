################################################################
################################################################
# Some simple and standard build settings.  The following are defined:
# 
# COMMON_INCLUDE_DIRS
# COMMON_LIBRARIES
# COMMON_LINK_DIRS
#
# To use this file, add "include( Common.cmake )" to the top of your
# CMakeLists.txt
#
################################################################
################################################################


################## AVOID BUILDING IN-SOURCE ####################
# Custom command to cleanup cmake polution when user accidently attempts an
# insoruce build.
ADD_CUSTOM_TARGET( cleansrc )
ADD_CUSTOM_COMMAND(
        TARGET  cleansrc
        COMMAND rm -rf `find . -name CMakeFiles -or -name Makefile -or -name cmake_install.cmake -or -name CMakeCache.txt`
        )
# Force good behavior: user shall not try an insource build.
if( ${CMAKE_BINARY_DIR} STREQUAL ${CMAKE_SOURCE_DIR} )
    MESSAGE( WARNING "\nERROR: You must build outside of the source tree.\n"
                         "       Recommend running 'make cleansrc' to clean the source tree.\n" )
endif()


################### SET BUILD TYPE OPTIONS ######################
# Verbose compile when debugging, and lots of optimization otherwise
IF( "${CMAKE_BUILD_TYPE}" STREQUAL "Debug" )
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -g -pg")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -g -pg")
ENDIF( "${CMAKE_BUILD_TYPE}" STREQUAL "Debug" )
 IF( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -Wall -Wextra -O3 -funroll-loops -finline-functions -mmmx -msse2 ")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wall -Wextra -O3 -funroll-loops -finline-functions -mmmx -msse2 ")
ENDIF( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )

############### Load CPATH and LIBRARY_PATH ##################
string( REPLACE ":" ";" COMMON_INCLUDE_DIRS "$ENV{CPATH}" ) 
string( REPLACE ":" ";" COMMON_LINK_DIRS "$ENV{LIBRARY_PATH}" ) 
list( REMOVE_DUPLICATES COMMON_INCLUDE_DIRS )
list( REMOVE_DUPLICATES COMMON_LINK_DIRS )

################################################################
# Add boost (really just for tr1 regex, which is missing in os x)
find_package( Boost COMPONENTS regex REQUIRED )

################################################################
# GetPot for command line parsing
#find_path( GETPOT_INCLUDE_DIR GetPot ${COMMON_INCLUDE_DIRS} REQUIRED )
#if( ${GETPOT_INCLUDE_DIR} STREQUAL "GETPOT_INCLUDE_DIR-NOTFOUND" )
#    message( FATAL_ERROR "GetPot header not found" )
#endif()
#list( APPEND COMMON_INCLUDE_DIRS ${GETPOT_INCLUDE_DIR} )

#message( STATUS "COMMON_INCLUDE_DIRS ${COMMON_INCLUDE_DIRS}" )
#message( STATUS "COMMON_LINK_DIRS ${COMMON_LINK_DIRS}" )
#message( STATUS "COMMON_LIBRARIES ${COMMON_LIBRARIES}" )

################################################################
set( COMMON_LIBRARIES 
        ${Boost_LIBRARIES}
   )

################################################################
# Make COMMON libs/headers available to all (overkill, but easy to use)
include_directories( ${COMMON_INCLUDE_DIRS}  )
link_directories( ${COMMON_LINK_DIRS} )
link_libraries( ${COMMON_LIBRARIES} )

# Hack to get eclipse working
if( APPLE )
    SET( CMAKE_ECLIPSE_EXECUTABLE /Applications/eclipse/eclipse CACHE STRING "" FORCE )
endif()

