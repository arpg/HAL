# - Try to find Occam Indigo SDK
# Once done, this will define
#
#  OccamIndigo_FOUND - system has libindigo
#  OccamIndigo_INCLUDE_DIRS - the Occam libindigo include directories
#  OccamIndigo_LIBRARIES - link these to use libindigo

include(LibFindMacros)

IF (UNIX)

# Include dir
    find_path(OccamIndigo_INCLUDE_DIR
            NAMES occam/indigo.h
            PATHS ${OccamIndigo_ROOT}/include /usr/local/include 
            )

# Finally the library itself
    find_library(OccamIndigo_LIBRARY
            NAMES indigo
            PATHS ${OccamIndigo_ROOT}/lib /usr/local/lib
	    )
ENDIF()

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(OccamIndigo_PROCESS_INCLUDES OccamIndigo_INCLUDE_DIR OccamIndigo_INCLUDE_DIRS)
set(OccamIndigo_PROCESS_LIBS OccamIndigo_LIBRARY OccamIndigo_LIBRARIES)
libfind_process(OccamIndigo)