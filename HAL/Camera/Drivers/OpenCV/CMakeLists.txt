find_package(OpenCV2 QUIET)

if(OpenCV2_FOUND)

    set( BUILD_OpenCV true CACHE BOOL force )

    if(BUILD_OpenCV)
        add_to_hal_sources(
            OpenCVDriver.h OpenCVDriver.cpp OpenCVFactory.cpp
        )
    endif()

endif()
