set(HDRS
    PosysDevice.h
    PosysDriverInterface.h
)

add_to_hal_headers( ${HDRS} )

add_subdirectory( Drivers )
