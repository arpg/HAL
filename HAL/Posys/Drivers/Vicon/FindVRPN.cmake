# Find the VRPN includes and library
#
# This module defines
# VRPN_INCLUDE_DIR
# VRPN_LIBRARIES
# VRPN_FOUND

find_path(VRPN_INCLUDE_DIR
    NAMES vrpn_Tracker.h
    PATHS
        /usr/local/include
        /usr/include
)

find_library( VRPN_LIBRARY
    NAMES vrpn
    PATHS
        /usr/local/lib
        /usr/lib
        /opt/local/lib
)

if( VRPN_INCLUDE_DIR AND VRPN_LIBRARY )
    set( VRPN_LIBRARIES ${VRPN_LIBRARY} )
    set( VRPN_INCLUDE_DIRS ${VRPN_INCLUDE_DIR} )
    set( VRPN_FOUND TRUE )
endif()
