find_package(OpenCV2 QUIET)

if(OpenCV2_FOUND)

    set( BUILD_OpenCV true CACHE BOOL force )

    if(BUILD_OpenCV)
        add_to_hal_libraries( ${OpenCV2_LIBRARIES} )
        add_to_hal_include_dirs( ${OpenCV2_INCLUDE_DIR} )
        add_to_hal_sources(
            OpenCVDriver.h OpenCVDriver.cpp OpenCVFactory.cpp
        )
    endif()

endif()
