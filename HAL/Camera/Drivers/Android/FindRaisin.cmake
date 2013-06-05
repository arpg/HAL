SET(RAISIN_POSSIBLE_ROOT_DIRS
	    "$ENV{RAISIN_ROOT_DIR}"                         # *NIX: custom install location (like ROS)
        /usr/local                                      # Linux: default dir by CMake
        /usr                                            # Linux
        /opt/local                                      # OS X: default MacPorts location
        ${CMAKE_INSTALL_PREFIX}                         # Android toolchain
)

FIND_PATH(RAISIN_ROOT_INC_DIR
          NAMES movidius-validation/validation-suite.h
          PATHS ${RAISIN_POSSIBLE_ROOT_DIRS}
          )

# absolute path to all libraries 
SET(RAISIN_LIBRARY_SEARCH_PATHS "${RAISIN_ROOT_DIR}/lib")

FIND_LIBRARY(RAISIN_VALIDATION_LIBRARY       NAMES validation-suite       PATHS ${RAISIN_LIBRARY_SEARCH_PATHS})
FIND_LIBRARY(RAISIN_FRAMEDATA_LIBRARY        NAMES frame_data              PATHS ${RAISIN_LIBRARY_SEARCH_PATHS})

SET(RAISIN_INCLUDE_DIRS
    ${RAISIN_VALIDATION_INCLUDE_DIR}
    ${RAISIN_FRAMEDATA_INCLUDE_DIR}
    )

SET(RAISIN_LIBRARIES
    ${RAISIN_VALIDATON_LIBRARY}
    ${RAISIN_FRAMEDATA_LIBRARY}
    )


SET(RAISIN_FOUND ON)
FOREACH(NAME ${RAISIN_INCLUDE_DIRS})
    IF(NOT EXISTS ${NAME})
        SET(RAISIN_FOUND OFF)
    ENDIF(NOT EXISTS ${NAME})
ENDFOREACH(NAME)
FOREACH(NAME ${RAISIN_LIBRARIES})
    IF(NOT EXISTS ${NAME})
        SET(RAISIN_FOUND OFF)
    ENDIF(NOT EXISTS ${NAME})
ENDFOREACH(NAME)

MARK_AS_ADVANCED(FORCE
                 RAISIN_ROOT_DIR
                 RAISIN_VALIDATION_INCLUDE_DIR
                 RAISIN_FRAMEDATA_INCLUDE_DIR
                 RAISIN_VALIDATION_LIBRARY
                 RAISIN_FRAMEDATA_LIBRARY
                 )

# display help message
IF(NOT RAISIN_FOUND)
    # make FIND_PACKAGE friendly
    MESSAGE(FATAL_ERROR "RAISIN not found. Please specify it's location with the RAISIN_ROOT_DIR env. variable.")
ENDIF(NOT RAISIN_FOUND)

