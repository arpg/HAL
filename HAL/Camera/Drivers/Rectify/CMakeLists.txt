find_package(Calibu QUIET)
if(Calibu_FOUND)
    add_to_hal_libraries( ${Calibu_LIBRARIES} )
    add_to_hal_include_dirs( ${Calibu_INCLUDE_DIRS} )
    add_to_hal_sources(
        RectifyDriver.h RectifyDriver.cpp RectifyFactory.cpp
    )
endif()
