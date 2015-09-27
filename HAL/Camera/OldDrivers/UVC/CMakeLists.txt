find_package(USB1 QUIET)

if(USB1_FOUND)

    set( BUILD_UVC true CACHE BOOL force )

    if( BUILD_UVC )
    
        message( STATUS "HAL: building 'UVC' camera driver.")

        add_to_hal_include_dirs(
            ${CMAKE_CURRENT_SOURCE_DIR}/libuvc/include
            ${USB1_INCLUDE_DIRS}
        )

        add_to_hal_libraries(
            ${USB1_LIBRARIES}
        )

        add_to_hal_sources(
            UvcDriver.h UvcDriver.cpp UvcFactory.cpp
            libuvc/src/ctrl.c libuvc/src/device.c libuvc/src/diag.c
            libuvc/src/frame.c libuvc/src/init.c libuvc/src/stream.c
            libuvc/src/misc.c
        )

    endif()
endif()
