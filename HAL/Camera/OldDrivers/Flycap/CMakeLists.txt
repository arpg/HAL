set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )


find_package( FLYCAP QUIET )

if( FLYCAP_FOUND )

    set( BUILD_Flycap true CACHE BOOL force )

    if( BUILD_Flycap )

        message( STATUS "HAL: building 'FlyCap' camera driver.")
        add_to_hal_include_dirs( ${FLYCAP_INCLUDE_DIR} )
        add_to_hal_libraries( ${FLYCAP_LIBRARIES} )
        add_to_hal_sources(
            FlycapDriver.h FlycapDriver.cpp FlycapFactory.cpp
        )
    endif()
endif()
