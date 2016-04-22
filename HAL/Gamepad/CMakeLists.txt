set(HDRS
		GamepadDevice.h
		GamepadDriverInterface.h
)

add_to_hal_headers( ${HDRS} )

add_subdirectory( Drivers )
