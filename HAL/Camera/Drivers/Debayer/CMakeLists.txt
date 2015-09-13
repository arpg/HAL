set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )

find_package( DC1394_2 QUIET)
if( DC1394_2_FOUND )

    message( STATUS "HAL: building 'Debayer' camera driver (using libdc1394).")
    add_to_hal_libraries( ${DC1394_2_LIBRARIES} )
    add_to_hal_include_dirs( ${DC1394_2_INCLUDE_DIR} )
    add_to_hal_sources(
        DebayerDriver.h DebayerDriver.cpp DebayerFactory.cpp
    )
endif()
