set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )


find_package( FLYCAP QUIET )

set( BUILD_Flycap true CACHE BOOL force )

if( FLYCAP_FOUND AND BUILD_DC1394 )
    add_to_hal_include_dirs( ${FLYCAP_INCLUDE_DIR} )
    add_to_hal_libraries( ${FLYCAP_LIBRARIES} )
    add_to_hal_sources(
        FlycapDriver.h FlycapDriver.cpp FlycapFactory.cpp
    )
endif()

