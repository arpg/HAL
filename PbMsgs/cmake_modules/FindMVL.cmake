# Find the Metric Vision Library
#
# This module defines
#  MVL_DIR          - MVL build directory.
#  MVL_INCLUDE_DIRS - Directories to find MVL and required headers.
#  MVL_LIBRARIES    - MVL libraries to link with (including full paths).
#  MVL_FOUND        - If false, do not try to use MVL.

# Be quite if already in cache.
IF( MVL_FOUND )
    SET( MVL_FIND_QUIETLY TRUE )
ENDIF( MVL_FOUND )

# mvl dependencies 
FIND_PACKAGE( CBLAS )
FIND_PACKAGE( CLAPACK )

# set define HAVE_APPLE_MATHS
IF( APPLE AND CLAPACK_FOUND )
		ADD_DEFINITIONS( -DHAVE_APPLE_MATHS )
ENDIF( APPLE AND CLAPACK_FOUND )


# First, try to find representative header.
FIND_PATH( MVL_INCLUDE_INSTALL_DIR mvl/image/image.h
	PATHS
    /usr/local/include
    /usr/include
    /opt/local/include
	ENV CPATH
)
SET( MVL_INCLUDE_INSTALL_DIR ${MVL_INCLUDE_INSTALL_DIR} CACHE INTERNAL "" FORCE )

# Found header dir, from which we can deduce the library dir
IF( NOT "${MVL_INCLUDE_INSTALL_DIR}" STREQUAL "MVL_INCLUDE_INSTALL_DIR-NOTFOUND" )
 
	# get the path.
    GET_FILENAME_COMPONENT( LIB_SEARCH_PATH ${MVL_INCLUDE_INSTALL_DIR} PATH )
    FIND_LIBRARY( MVL_LIBRARY NAMES mvl 
            PATHS ${LIB_SEARCH_PATH}/lib ENV CPATH )

	IF( MVL_LIBRARY )
        FIND_LIBRARY( LIBZ_LIBRARIES NAMES z )
        IF( "${LIBZ_LIBRARIES}" STREQUAL "LIBZ_LIBRARIES-NOTFOUND" )
            MESSAGE( FATAL_ERROR "MVL requires libz, please install it" ) 
        ENDIF( "${LIBZ_LIBRARIES}" STREQUAL "LIBZ_LIBRARIES-NOTFOUND" )
        SET( LIBZ_LIBRARIES ${LIBZ_LIBRARIES} CACHE INTERNAL "" FORCE )

        # Strip filename
		GET_FILENAME_COMPONENT( MVL_LIBRARY_DIR ${MVL_LIBRARY} PATH  )
 
        SET( MVL_LIBRARY_DIR ${MVL_LIBRARY_DIR} CACHE PATH "" FORCE )
#		SET( MVL_INCLUDE_DIRS ${MVL_INCLUDE_INSTALL_DIR} CACHE PATH "" FORCE )
#        SET( MVL_LIBRARIES ${MVL_LIBRARIES} ${LIBZ_LIBRARIES} ) # zlib
		SET( MVL_INCLUDE_DIRS 
                ${MVL_INCLUDE_INSTALL_DIR} 
                ${BLAS_INCLUDE_DIR} 
                ${LAPACK_INCLUDE_DIR} 
                CACHE PATH "" FORCE )
        SET( MVL_LIBRARIES 
                ${MVL_LIBRARY} 
                mvlargparse
                ${BLAS_LIBRARIES}
                ${CLAPACK_LIBRARIES} 
                ${LIBZ_LIBRARIES}
                )
	ENDIF( MVL_LIBRARY )

	MARK_AS_ADVANCED( MVL_LIBRARY_DIR )
	MARK_AS_ADVANCED( MVL_INCLUDE_DIRS )
	MARK_AS_ADVANCED( MVL_LIBRARIES )
ENDIF( NOT "${MVL_INCLUDE_INSTALL_DIR}" STREQUAL "MVL_INCLUDE_INSTALL_DIR-NOTFOUND" )

IF( MVL_INCLUDE_DIRS AND MVL_LIBRARY_DIR )
    SET( MVL_FOUND TRUE CACHE INTERNAL "" )
ELSE( MVL_INCLUDE_DIRS AND MVL_LIBRARY_DIR )
    SET( MVL_FOUND FALSE CACHE INTERNAL "" )
ENDIF( MVL_INCLUDE_DIRS AND MVL_LIBRARY_DIR )



IF( MVL_FOUND )
    IF( NOT MVL_FIND_QUIETLY )
        MESSAGE( STATUS "Looking for MVL headers -- found" )
        MESSAGE( STATUS "Looking for MVL libraries -- found" )
    ENDIF( NOT MVL_FIND_QUIETLY )
ELSE( MVL_FOUND )
    IF( MVL_REQUIRED )
        MESSAGE( FATAL_ERROR "Could NOT find MVL -- please provide the directory containing MVL.")
    ELSE( MVL_REQUIRED)
        MESSAGE( STATUS "Looking for MVL library -- not found" )
    ENDIF( MVL_REQUIRED)
ENDIF( MVL_FOUND )

MARK_AS_ADVANCED( MVL_FOUND )
MARK_AS_ADVANCED( MVL_INCLUDE_DIRS )
MARK_AS_ADVANCED( MVL_LIBRARY_DIR )
MARK_AS_ADVANCED( MVL_LIBRARIES )

