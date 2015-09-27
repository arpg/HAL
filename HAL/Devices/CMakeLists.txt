set(HDRS
    DriverFactory.h
    DeviceDriverRegistry.h
    DeviceTime.h
    DeviceDriverInterface.h
)

add_to_hal_sources( DeviceTime.cpp DeviceDriverRegistry.cpp )

add_to_hal_headers( ${HDRS} )
