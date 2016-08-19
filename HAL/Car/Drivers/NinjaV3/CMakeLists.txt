set(BUILD_Car false CACHE BOOL force)

if(BUILD_Car)
#  find_package(PbMsgs QUIET)

#  if(PbMsgs_FOUND)
    set(DRIVER_SOURCES NinjaV3CarDriver.cpp)
    set(DRIVER_HEADERS NinjaV3CarDriver.h ComDriver.h)
    # add_to_hal_libraries add_to_hal_include_dirs not called as NinjaV3Car
    # would have added Node related values.
#    message("PbMsgs: ${PbMsgs_INCLUDE_DIRS}" )
    add_to_hal_sources(NinjaV3CarDriver.h NinjaV3CarDriver.cpp NinjaV3CarFactory.cpp ComDriver.h)
#    add_to_hal_libraries( ${Node_LIBRARIES} )
#    add_to_hal_include_dirs( ${Node_INCLUDE_DIRS} ${PbMsgs_INCLUDE_DIRS} )
#  endif()

endif()
