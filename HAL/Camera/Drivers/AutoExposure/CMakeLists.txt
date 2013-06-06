find_package(USB1 QUIET)

if(USB1_FOUND)
    add_to_hal_sources(
        AutoExposureDriver.h AutoExposureDriver.cpp AutoExposureFactory.cpp
    )
endif()
