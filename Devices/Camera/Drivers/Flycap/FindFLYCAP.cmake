#
#  Try to find Flycap library
# 
#  This module defines the following variables:
#      FLYCAP_FOUND       -
#      FLYCAP_INCLUDE_DIR - include directory
#      FLYCAP_LIBRARIES   - full path libraries
#   


IF( NOT FLYCAP_FOUND )
  # One variable to hold all the paths that are used for searching below.
  # Easier to maintain, to debug and to extend...
  # Note, CMake does not complain about path that do not exist...
  SET( FLYCAP_SEARCH_PATHS
        /usr/include/flycapture
        $ENV{HOME}/lib 
        $ENV{HOME}/include
        /usr/local/lib 
        /usr/local/include
        /usr/lib 
        /usr/include
        $ENV{FLYCAP_HOME}/lib
        $ENV{FLYCAP_HOME}/include
        $ENV{LD_LIBRARY_PATH} 
        $ENV{CPATH}
        ${CMAKE_FRAMEWORK_PATH}
     )

  # Common for both UNIX and APPLE.
  FIND_LIBRARY( FLYCAP_LIBRARY NAMES flycapture PATHS ${FLYCAP_SEARCH_PATHS} )
  FIND_PATH( FLYCAP_INCLUDE_DIR flycapture/FlyCapture2.h ${FLYCAP_SEARCH_PATHS} )

  IF( NOT ${FLYCAP_LIBRARY} STREQUAL "FLYCAP_LIBRARY-NOTFOUND" )

    SET( FLYCAP_FOUND TRUE )

  ENDIF( NOT ${FLYCAP_LIBRARY} STREQUAL "FLYCAP_LIBRARY-NOTFOUND" )

  # Only advance if successful...
  IF( FLYCAP_FOUND )
      # Nice to have.
      SET(FLYCAP_FOUND true CACHE internal "" )
      SET(FLYCAP_LIBRARIES flycapture CACHE STRING "Libraries needed for PointGrey cameras to work." FORCE)
      MARK_AS_ADVANCED(
          FLYCAP_LIBRARY
          FLYCAP_FOUND
          FLYCAP_INCLUDE_DIR
          FLYCAP_LIBRARIES
      )
  ELSE(FLYCAP_FOUND)
      SET(FLYCAP_FOUND false CACHE internal "" )
  ENDIF(FLYCAP_FOUND)

ENDIF( NOT FLYCAP_FOUND )


