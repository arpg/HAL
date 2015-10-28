set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )

find_package( PHIDGETS QUIET )

if( PHIDGETS_FOUND )

    set( BUILD_Phidgets true CACHE BOOL force )
    if( BUILD_Phidgets )

        add_to_hal_include_dirs( ${PHIDGETS_INCLUDE_DIR} )
        add_to_hal_libraries( ${PHIDGETS_LIBRARIES} )
        add_to_hal_sources( PhidgetsDriver.h PhidgetsDriver.cpp PhidgetsFactory.cpp )
    endif()
endif()
