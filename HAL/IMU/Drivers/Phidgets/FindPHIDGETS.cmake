# Find the Phidgets includes and library
#
# This module defines
# PHIDGETS_INCLUDE_DIR
# PHIDGETS_LIBRARIES
# PHIDGETS_FOUND

find_path(PHIDGETS_INCLUDE_DIR
    NAMES phidget22.h
    PATHS
        /usr/local/include
        /usr/include
        /Library/Frameworks/Phidget22.framework/Headers
)

find_library( PHIDGETS_LIBRARY
    NAMES Phidget22
    PATHS
        /usr/local/lib
        /usr/lib
        /opt/local/lib
        /Library/Frameworks/Phidget22.framework
)

if( PHIDGETS_INCLUDE_DIR AND PHIDGETS_LIBRARY )
    set( PHIDGETS_LIBRARIES ${PHIDGETS_LIBRARY} )
    set( PHIDGETS_INCLUDE_DIRS ${PHIDGETS_INCLUDE_DIR} )
    set( PHIDGETS_FOUND TRUE )
endif()
