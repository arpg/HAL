set(HDRS
    CarDevice.h
    CarDriverInterface.h
)

add_to_hal_headers( ${HDRS} )

add_subdirectory(Drivers)
