
set( BUILD_FileReader true CACHE BOOL force )

find_package(OpenCV QUIET COMPONENTS core)

set(OpenCV_FOUND FALSE)
if(OpenCV_VERSION_MAJOR EQUAL 2)
    find_package(OpenCV QUIET COMPONENTS core imgproc highgui)
elseif(OpenCV_VERSION_MAJOR EQUAL 3)
    find_package(OpenCV QUIET COMPONENTS core imgproc imgcodecs videoio highgui)
endif()


if( BUILD_FileReader AND OpenCV_FOUND)

    message( STATUS "HAL: building 'FileReader' camera driver.")
    add_to_hal_libraries( ${OpenCV_LIBS} )
    add_to_hal_include_dirs( ${OpenCV_INCLUDE_DIRS} )
    add_to_hal_sources(
        FileReaderDriver.cpp FileReaderFactory.cpp ReadImage.cpp
        FileReaderDriver.h ReadImage.h
    )

endif()
