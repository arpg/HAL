find_package(OpenCV2 QUIET)

if(OpenCV2_FOUND)

    set( BUILD_Webcam true CACHE BOOL force )

    if(BUILD_Webcam)
        add_to_hal_sources(
            WebcamDriver.h WebcamDriver.cpp WebcamFactory.cpp
        )
    endif()

endif()
