find_package(OpenCV QUIET COMPONENTS core imgproc)

if(OpenCV_FOUND)

    message( STATUS "HAL: building 'Convert' abstract camera driver (using libopencv).")

    add_to_hal_libraries( ${OpenCV_LIBS} )
    add_to_hal_include_dirs( ${OpenCV_INCLUDE_DIRS} )
    add_to_hal_sources(
        ConvertDriver.h ConvertDriver.cpp ConvertFactory.cpp
    )
endif()
