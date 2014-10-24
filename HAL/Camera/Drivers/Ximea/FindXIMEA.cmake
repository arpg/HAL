#
##  Try to find ximea library
##
##  This module defines the following variables:
##      XIMEA_FOUND       -
##      XIMEA_INCLUDE_DIR - include directory
##      XIMEA_LIBRARIES   - full path libraries
##

SET( XIMEA_SEARCH_PATHS
    /opt/XIMEA/lib
    /opt/XIMEA/include
    /Library/Frameworks/m3api.framework/Libraries
)

find_path(XIMEA_INCLUDE_DIR
    NAMES xiApi.h
    PATHS ${XIMEA_SEARCH_PATHS}
)

find_library( XIMEA_LIBRARY
    NAMES libXIMEA_GenTL.dylib
    PATHS ${XIMEA_SEARCH_PATHS}
)

if( XIMEA_INCLUDE_DIR AND XIMEA_LIBRARY )
    set( XIMEA_LIBRARIES ${XIMEA_LIBRARY} )
    set( XIMEA_INCLUDE_DIRS ${XIMEA_INCLUDE_DIR} )
    set( XIMEA_FOUND TRUE )
endif()

