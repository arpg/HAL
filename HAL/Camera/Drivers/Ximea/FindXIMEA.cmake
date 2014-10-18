#
#  Try to find ximea library
# 
#  This module defines the following variables:
#      XIMEA_FOUND       -
#      XIMEA_INCLUDE_DIR - include directory
#      XIMEA_LIBRARIES   - full path libraries
#   


IF( NOT XIMEA_FOUND )
  # One variable to hold all the paths that are used for searching below.
  # Easier to maintain, to debug and to extend...
  # Note, CMake does not complain about path that do not exist...
  SET( XIMEA_SEARCH_PATHS
        /opt/XIMEA/lib 
        /opt/XIMEA/include
        ${CMAKE_FRAMEWORK_PATH}/Versions/Current/Headers
        ${CMAKE_FRAMEWORK_PATH}/Versions/Current/Libraries
     )

  # Common for both UNIX and APPLE.
  FIND_LIBRARY( XIMEA_LIBRARY NAMES m3api PATHS ${XIMEA_SEARCH_PATHS} )
  FIND_PATH( XIMEA_INCLUDE_DIR xiApi.h ${XIMEA_SEARCH_PATHS} )

IF( NOT ${XIMEA_LIBRARY} STREQUAL "XIMEA_LIBRARY-NOTFOUND" )

    SET( XIMEA_FOUND TRUE )

  ENDIF( NOT ${XIMEA_LIBRARY} STREQUAL "XIMEA_LIBRARY-NOTFOUND" )

  # Only advance if successful...
  IF( XIMEA_FOUND )
      # Nice to have.
      SET(XIMEA_FOUND true CACHE internal "" )
      SET(XIMEA_LIBRARIES ximea CACHE STRING "Libraries needed for ximea cameras to work." FORCE)
      MARK_AS_ADVANCED(
          XIMEA_LIBRARY
          XIMEA_FOUND
          XIMEA_INCLUDE_DIR
          XIMEA_LIBRARIES
      )
  ELSE(XIMEA_FOUND)
      SET(XIMEA_FOUND false CACHE internal "" )
  ENDIF(XIMEA_FOUND)

ENDIF( NOT XIMEA_FOUND )


