find_package(OpenCV2 QUIET)

set( BUILD_Webcam true CACHE BOOL force )

if(BUILD_Webcam AND OpenCV2_FOUND)
    add_to_hal_sources(
        WebcamDriver.h WebcamDriver.cpp WebcamFactory.cpp
    )
endif()

