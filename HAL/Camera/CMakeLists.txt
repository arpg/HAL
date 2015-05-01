set(HDRS
    CameraDevice.h
    CameraDriverInterface.h
)

add_to_hal_headers( ${HDRS} )

add_subdirectory( Drivers )
