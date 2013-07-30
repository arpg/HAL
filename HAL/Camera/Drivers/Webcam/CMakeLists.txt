find_package(OpenCV2 QUIET)

if(OpenCV2_FOUND)
    add_to_hal_sources(
        WebcamDriver.h WebcamDriver.cpp WebcamFactory.cpp
    )
endif()

