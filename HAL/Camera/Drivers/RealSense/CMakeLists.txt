find_package(USB1 QUIET)

if(USB1_FOUND AND BUILD_UVC)

    set( BUILD_RealSense true CACHE BOOL force )

    if( BUILD_RealSense)
        add_to_hal_include_dirs(
            ${USB1_INCLUDE_DIRS}
        )

        add_to_hal_libraries(
            ${USB1_LIBRARIES}
        )

        add_to_hal_sources(
            RealSense.h RealSense.cpp RealSenseFactory.cpp
        )

    endif()
endif()
