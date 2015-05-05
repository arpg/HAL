set(HDRS
    LIDARDevice.h
    LIDARDriverInterface.h
)

add_to_hal_headers( ${HDRS} )

add_subdirectory( Drivers )
