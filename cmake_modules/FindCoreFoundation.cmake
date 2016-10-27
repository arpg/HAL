# Locate Apple CoreFoundation
# This module defines
# COREFOUNDATION_LIBRARY
# COREFOUNDATION_FOUND, if false, do not try to link to gdal 
# COREFOUNDATION_INCLUDE_DIR, where to find the headers
#
# $COREFOUNDATION_DIR is an environment variable that would
# correspond to the ./configure --prefix=$COREFOUNDATION_DIR
#
# Created by Christian Frisson.

IF(APPLE)
  FIND_PATH(COREFOUNDATION_INCLUDE_DIR CoreFoundation/CoreFoundation.h)
  FIND_LIBRARY(COREFOUNDATION_LIBRARY CoreFoundation)
ENDIF()


SET(COREFOUNDATION_FOUND "NO")
IF(COREFOUNDATION_LIBRARY AND COREFOUNDATION_INCLUDE_DIR)
  SET(COREFOUNDATION_FOUND "YES")
ENDIF()

