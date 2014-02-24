find_package(OpenCV2 QUIET)

if(OpenCV2_FOUND)
    add_to_hal_libraries( ${OpenCV2_LIBRARIES} )
    add_to_hal_include_dirs( ${OpenCV2_INCLUDE_DIR} )
    add_to_hal_sources(
        ConvertDriver.h ConvertDriver.cpp ConvertFactory.cpp
    )
endif()
