# - Try to find Aravis
# Once done, this will define
#
#  Aravis_FOUND - system has library
#  Aravis_INCLUDE_DIRS - the  include directories
#  Aravis_LIBRARIES - link these to use the library

include(LibFindMacros)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_Aravis REQUIRED aravis-0.6)

find_path(ARAVIS_INCLUDE_DIR aravis-0.6/arv.h
          HINTS ${PC_Aravis_INCLUDEDIR} ${PC_Aravis_INCLUDE_DIRS}
          PATH_SUFFIXES aravis )

find_library(ARAVIS_LIBRARY NAMES ${PC_Aravis_LIBRARIES}
             HINTS ${PC_Aravis_LIBDIR} ${PC_Aravis_LIBRARY_DIRS} )

include(FindPackageHandleStandardArgs) 

find_package_handle_standard_args(Aravis-0.6  DEFAULT_MSG
                                  ARAVIS_LIBRARY ARAVIS_INCLUDE_DIR)
				  
mark_as_advanced(ARAVIS_INCLUDE_DIR ARAVIS_LIBRARY )			  
set(Aravis_LIBRARIES ${ARAVIS_LIBRARY} )
set(Aravis_INCLUDE_DIRS ${ARAVIS_INCLUDE_DIR} )
set(Aravis_FOUND ${PC_Aravis_FOUND})

#message("Aravis libs: ${Aravis_LIBRARIES}")
#message("Aravis found: ${Aravis_FOUND}")