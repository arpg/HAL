# FindRaisin.cmake
#   Raisin_FOUND
#   Raisin_INCLUDE_DIRS
#   Raisin_LIBRARIES

SET(Raisin_POSSIBLE_ROOT_DIRS
	"$ENV{Raisin_ROOT_DIR}"                         # *NIX: custom install location (like ROS)
        /usr/local                                      # Linux: default dir by CMake
        /usr                                            # Linux
        /opt/local                                      # OS X: default MacPorts location
        ${CMAKE_INSTALL_PREFIX}                         # Android toolchain
)

FIND_PATH(Raisin_ROOT_INC_DIR
          NAMES movidius-validation/validation-suite.h
          PATHS ${Raisin_POSSIBLE_ROOT_DIRS}
          )

# absolute path to all libraries 
SET(Raisin_LIBRARY_SEARCH_PATHS "${Raisin_ROOT_DIR}/lib")

FIND_LIBRARY(Raisin_VALIDATION_LIBRARY       NAMES validation-suite       PATHS ${Raisin_LIBRARY_SEARCH_PATHS})
FIND_LIBRARY(Raisin_FRAMEDATA_LIBRARY        NAMES frame_data              PATHS ${Raisin_LIBRARY_SEARCH_PATHS})

SET(Raisin_INCLUDE_DIRS
    ${Raisin_VALIDATION_INCLUDE_DIR}
    ${Raisin_FRAMEDATA_INCLUDE_DIR}
    )

SET(Raisin_LIBRARIES
    ${Raisin_VALIDATON_LIBRARY}
    ${Raisin_FRAMEDATA_LIBRARY}
    )


SET(Raisin_FOUND ON)
FOREACH(NAME ${Raisin_INCLUDE_DIRS})
    IF(NOT EXISTS ${NAME})
        SET(Raisin_FOUND OFF)
    ENDIF(NOT EXISTS ${NAME})
ENDFOREACH(NAME)
FOREACH(NAME ${Raisin_LIBRARIES})
    IF(NOT EXISTS ${NAME})
        SET(Raisin_FOUND OFF)
    ENDIF(NOT EXISTS ${NAME})
ENDFOREACH(NAME)

MARK_AS_ADVANCED(FORCE
                 Raisin_ROOT_DIR
                 Raisin_VALIDATION_INCLUDE_DIR
                 Raisin_FRAMEDATA_INCLUDE_DIR
                 Raisin_VALIDATION_LIBRARY
                 Raisin_FRAMEDATA_LIBRARY
                 )

IF(Raisin_FOUND)
   IF(NOT Raisin_FIND_QUIETLY)
      MESSAGE(STATUS "Found Raisin: ${Raisin_LIBRARY}")
   ENDIF (NOT Raisin_FIND_QUIETLY)
ELSE(Raisin_FOUND)
   IF(Raisin_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find Raisin. Please specify it's location with the Raisin_ROOT_DIR env. variable.")
   ENDIF(Raisin_FIND_REQUIRED)
ENDIF(Raisin_FOUND)
