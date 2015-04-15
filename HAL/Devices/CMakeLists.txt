set(HDRS
    DeviceException.h
    DeviceFactory.h
    DeviceRegistry.h
    DeviceTime.h
    DriverInterface.h
    SharedLoad.h
)

add_to_hal_sources( DeviceTime.cpp DeviceRegistry.cpp )

add_to_hal_headers( ${HDRS} )
