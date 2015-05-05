set(HDRS
    EncoderDevice.h
    EncoderDriverInterface.h
)

add_to_hal_headers( ${HDRS} )

add_subdirectory( Drivers )
