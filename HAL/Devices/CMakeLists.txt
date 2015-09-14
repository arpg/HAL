set(HDRS
    DeviceException.h
    DriverFactory.h
    DeviceRegistry.h
    DeviceTime.h
    DriverInterface.h
)

add_to_hal_sources( DeviceTime.cpp DeviceRegistry.cpp )

add_to_hal_headers( ${HDRS} )
