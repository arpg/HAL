if( OpenCV2_FOUND )

    set( BUILD_OpenCV true CACHE BOOL force )

    if( BUILD_OpenCV )

        message( STATUS "HAL: building 'OpenCV' driver.")
        add_to_hal_sources(
            OpenCVDriver.h OpenCVDriver.cpp OpenCVFactory.cpp
        )
    endif()
endif()
