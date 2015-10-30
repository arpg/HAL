set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )

find_package( DC1394_2 QUIET )

if( DC1394_2_FOUND )

    set( BUILD_DC1394 true CACHE BOOL force )

    if( BUILD_DC1394 )

        message( STATUS "HAL: building 'DC1394' (firewire) camera driver.")
        add_to_hal_libraries( ${DC1394_2_LIBRARIES} )
        add_to_hal_include_dirs( ${DC1394_2_INCLUDE_DIR} )
        add_to_hal_sources(
            DC1394Driver.h DC1394Driver.cpp DC1394Factory.cpp
        )
    endif()
endif()
