set(HDRS
    DeviceException.h
    DeviceFactory.h
    DeviceRegistry.h
    DeviceTime.h
    DriverInterface.h
    SharedLoad.h
)

add_to_hal_sources(
    ${HDRS} DeviceTime.cpp DeviceRegistry.cpp
)

install(FILES ${HDRS}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/HAL/Devices
)
