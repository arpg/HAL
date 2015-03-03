# - Try to find libzmqpp
# Once done, this will define
#
#  ZeroMQPP_FOUND - system has libzmqpp
#  ZeroMQPP_INCLUDE_DIRS - the libzmqpp include directories
#  ZeroMQPP_LIBRARIES - link these to use libzmqpp

include(LibFindMacros)

IF (UNIX)

# Include dir
    find_path(ZeroMQPP_INCLUDE_DIR
            NAMES zmqpp/zmqpp.hpp
            PATHS ${ZEROMQPP_ROOT}/include /usr/local/include 
            )

# Finally the library itself
    find_library(ZeroMQPP_LIBRARY
            NAMES zmqpp
            PATHS ${ZEROMQPP_ROOT}/lib /usr/local/lib
            )
ELSEIF (WIN32)
    find_path(ZeroMQPP_INCLUDE_DIR
            NAMES zmqpp.hpp
            PATHS ${ZEROMQPP_ROOT}/include ${CMAKE_INCLUDE_PATH}
            )
# Finally the library itself
    find_library(ZeroMQPP_LIBRARY
            NAMES libzmqpp
            PATHS ${ZEROMQPP_ROOT}/lib ${CMAKE_LIB_PATH}
            )
ENDIF()

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(ZeroMQPP_PROCESS_INCLUDES ZeroMQPP_INCLUDE_DIR ZeroMQPP_INCLUDE_DIRS)
set(ZeroMQPP_PROCESS_LIBS ZeroMQPP_LIBRARY ZeroMQPP_LIBRARIES)
libfind_process(ZeroMQPP)

