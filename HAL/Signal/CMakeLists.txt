set(HDRS
    SignalDevice.h
    SignalDriverInterface.h
)

add_to_hal_headers( ${HDRS} )

add_subdirectory( Drivers )
