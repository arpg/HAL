# - Try to find peak-linux driver
# Once done, this will define
#
#  ZeroMQ_FOUND - system has pcan
#  ZeroMQ_INCLUDE_DIRS - the libpcan include directories

include(LibFindMacros)

IF(UNIX)
# Include dir
    find_path(PCAN_INCLUDE_DIR
            NAMES libpcan.h
            PATHS /usr/include ${PCAN_PKGCONF_INCLUDE_DIRS}
            )
ENDIF()

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
    set(PCAN_PROCESS_INCLUDES PCAN_INCLUDE_DIR PCAN_INCLUDE_DIRS)
libfind_process(PCAN)
