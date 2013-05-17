find_library(LIBUSB_LIBRARY_NAMES usb-1.0
	PATHS /opt/local/lib)

find_path(LIBUSB_INCLUDE_DIR libusb-1.0/libusb.h
	PATHS /opt/local/include)

add_to_hal_include_dirs(
    ${CMAKE_CURRENT_SOURCE_DIR}/libuvc/include
    ${LIBUSB_INCLUDE_DIR}
)

add_to_hal_libraries(
    ${LIBUSB_LIBRARY_NAMES}
)

add_to_hal_sources(
    UvcDriver.h UvcDriver.cpp UvcFactory.cpp
    libuvc/src/ctrl.c libuvc/src/device.c libuvc/src/diag.c
    libuvc/src/frame.c libuvc/src/init.c libuvc/src/stream.c
    libuvc/src/misc.c
)
