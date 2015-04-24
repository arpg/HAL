set(HDRS
    IMUDevice.h
    IMUDriverInterface.h
)

add_to_hal_headers( ${HDRS} )

add_subdirectory( Drivers )
