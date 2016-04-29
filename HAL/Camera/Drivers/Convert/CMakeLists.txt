
message( STATUS "HAL: building 'Convert' abstract camera driver (using libopencv).")
find_package(OpenCV QUIET COMPONENTS core)

set(OpenCV_FOUND FALSE)
if(OpenCV_VERSION_MAJOR EQUAL 2)
    find_package(OpenCV QUIET COMPONENTS core imgproc highgui)
elseif(OpenCV_VERSION_MAJOR EQUAL 3)
    find_package(OpenCV QUIET COMPONENTS core imgproc imgcodecs videoio)
endif()

add_to_hal_libraries( ${OpenCV_LIBS} )
add_to_hal_include_dirs( ${OpenCV_INCLUDE_DIRS} )
add_to_hal_sources(
    ConvertDriver.h ConvertDriver.cpp ConvertFactory.cpp
)
