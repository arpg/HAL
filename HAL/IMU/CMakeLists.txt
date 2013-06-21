set(HDRS
    IMUDevice.h
    IMUDriverInterface.h
)

add_to_hal_sources( ${HDRS} )

install(FILES ${HDRS}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/HAL/IMU
)

add_subdirectory( Drivers )
