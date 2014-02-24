#
#  Try to find libDC1394 verson 2
# 
#  This module defines the following variables:
#      DC1394_2_FOUND       - 
#      DC1394_2_INCLUDE_DIR - include directory
#      DC1394_2_LIBRARIES   - full path libraries
#   
# Jan Woetzel www.mip.informatik.uni-kiel.de/~jw
#

IF( NOT DC1394_2_FOUND )
  # One variable to hold all the paths that are used for searching below.
  # Easier to maintain, to debug and to extend...
  # Note, CMake does not complain about path that do not exist...
  SET( DC1394_2_SEARCH_PATHS
        $ENV{HOME}/lib 
        $ENV{HOME}/include
        $ENV{HOME}/code/libdc1394/lib 
        $ENV{HOME}/Code/libdc1394/lib
        $ENV{HOME}/code/libdc1394/include 
        $ENV{HOME}/Code/libdc1394/include
        /usr/local/lib 
        /usr/local/include
        /usr/lib 
        /usr/include
        $ENV{DC1394_2_HOME}/lib 
        $ENV{DC1394_2_HOME}/include
        $ENV{LD_LIBRARY_PATH} 
        $ENV{CPATH}
        ${CMAKE_FRAMEWORK_PATH}
     )

  # Common for both UNIX and APPLE.
  FIND_LIBRARY( DC1394_2_LIBRARY NAMES dc1394 PATHS ${DC1394_2_SEARCH_PATHS} )
  FIND_PATH( DC1394_2_INCLUDE_DIR dc1394/control.h ${DC1394_2_SEARCH_PATHS} )

  IF( NOT ${DC1394_2_LIBRARY} STREQUAL "DC1394_2_LIBRARY-NOTFOUND" )

      IF( UNIX AND NOT APPLE )
          # Using DC1394_2_RAW1394_LIB instead of just RAW1394_LIB automatically
          # groups all variables dealing with 1394 libs together...
          FIND_LIBRARY( DC1394_2_RAW1394_LIB NAMES raw1394 PATHS ${DC1394_2_SEARCH_PATHS} )
          SET( DC1394_2_LIBRARIES ${DC1394_2_LIBRARY} ${DC1394_2_RAW1394_LIB} )
          SET( DC1394_2_FOUND TRUE )
      ENDIF()

      # hack for libusb-1.0, WTF!
      IF( LINUX )
          SET(DC1394_2_LIBRARIES ${DC1394_2_LIBRARY} "usb-1.0")
      ENDIF()
      IF( APPLE )
          SET(DC1394_2_LIBRARIES ${DC1394_2_LIBRARY} 
                  "-framework\\ IOKit"
                  "-framework\\ CoreFoundation"
                  "-framework\\ Cocoa")
          SET( DC1394_2_FOUND TRUE )
      ENDIF( APPLE )

  ENDIF( NOT ${DC1394_2_LIBRARY} STREQUAL "DC1394_2_LIBRARY-NOTFOUND" ) 

  # Only advance if successful...
  IF( DC1394_2_FOUND )
      # Nice to have.
      SET(DC1394_2_FOUND true CACHE internal "" )
      SET(DC1394_2_LIBRARIES ${DC1394_2_LIBRARIES} CACHE STRING "Libraries needed for 1394/firewire to work." FORCE)
      MARK_AS_ADVANCED(
          DC1394_2_LIBRARY
          DC1394_2_RAW1394_LIB
          DC1394_2_FOUND
          DC1394_2_INCLUDE_DIR
          DC1394_2_LIBRARIES
      )
  ELSE(DC1394_2_FOUND)
      SET(DC1394_2_FOUND false CACHE internal "" )
  ENDIF(DC1394_2_FOUND)

ENDIF( NOT DC1394_2_FOUND )


