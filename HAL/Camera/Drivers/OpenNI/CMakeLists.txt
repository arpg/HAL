set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )

find_package( OpenNI QUIET )

if( OPENNI_FOUND )

    set( BUILD_OpenNI true CACHE BOOL force )

    if( BUILD_OpenNI )

        message( STATUS "HAL: building 'OpenNI' camera driver.")
        add_to_hal_include_dirs( ${OPENNI_INCLUDE_DIRS} )
        add_to_hal_libraries( ${OPENNI_LIBRARY} )
        add_to_hal_sources( OpenNIDriver.h OpenNIDriver.cpp OpenNIFactory.cpp )
    endif()
endif()
