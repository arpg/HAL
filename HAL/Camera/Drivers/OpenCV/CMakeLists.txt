find_package(OpenCV QUIET COMPONENTS core)

set(OpenCV_FOUND FALSE)
if(OpenCV_VERSION_MAJOR EQUAL 2)
    find_package(OpenCV QUIET COMPONENTS core imgproc highgui)
elseif(OpenCV_VERSION_MAJOR EQUAL 3)
    find_package(OpenCV QUIET COMPONENTS core imgproc imgcodecs videoio)
endif()

if(OpenCV_FOUND)

    set( BUILD_OpenCV true CACHE BOOL force )

    if(BUILD_OpenCV)

        message( STATUS "HAL: building 'OpenCV' camera driver.")
        add_to_hal_libraries( ${OpenCV_LIBS} )
        add_to_hal_include_dirs( ${OpenCV_INCLUDE_DIRS} )
        add_to_hal_sources(
            OpenCVDriver.h OpenCVDriver.cpp OpenCVFactory.cpp
        )
    endif() 

endif()
