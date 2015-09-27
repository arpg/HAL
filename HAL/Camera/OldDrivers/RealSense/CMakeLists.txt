find_package(USB1 QUIET)

option( BUILD_RealSense "Build RealSense Driver." OFF )

if(USB1_FOUND AND BUILD_RealSense) # to be added: AND BUILD_UVC

        message( STATUS "HAL: building 'RealSense' camera driver.")

        add_to_hal_include_dirs(
            ${CMAKE_CURRENT_SOURCE_DIR}/libuvc/include
            ${USB1_INCLUDE_DIRS}
        )

        add_to_hal_libraries(
            ${USB1_LIBRARIES}
        )

        add_to_hal_sources(
            RealSense.h RealSense.cpp RealSenseFactory.cpp
            libuvc_ex/src/ctrl.c libuvc_ex/src/ctrl-gen.c libuvc_ex/src/device.c libuvc_ex/src/diag.c
            libuvc_ex/src/frame.c libuvc_ex/src/init.c libuvc_ex/src/stream.c
            libuvc_ex/src/misc.c
        )

endif()
