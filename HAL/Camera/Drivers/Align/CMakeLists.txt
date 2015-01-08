find_package(OpenCV2 QUIET)
find_package(OpenCL QUIET)

if(OpenCV2_FOUND)
    add_to_hal_libraries( ${OpenCV2_LIBRARIES} )
    add_to_hal_include_dirs( ${OpenCV2_INCLUDE_DIR} ${OPENCL_INCLUDE_DIRS} )
    add_to_hal_sources(
        AlignDriver.h AlignDriver.cpp AlignFactory.cpp 
        depth_registration.h depth_registration.cpp
        depth_registration_cpu.h depth_registration_cpu.cpp
    )
    if(OPENCL_FOUND)
        add_to_hal_definitions("-DWITH_OPENCL_SUPPORT")
        add_to_hal_definitions("-DREG_OPENCL_FILE=\"${CMAKE_CURRENT_SOURCE_DIR}/depth_registration.cl\"")
        add_to_hal_libraries( ${OPENCL_LIBRARIES} )
        add_to_hal_include_dirs( ${OPENCL_INCLUDE_DIRS} )
        add_to_hal_sources(
            depth_registration_opencl.h depth_registration_opencl.cpp
            depth_registration.cl
        )
    endif()
endif()
