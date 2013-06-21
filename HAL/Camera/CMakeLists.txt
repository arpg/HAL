set(HDRS
    CameraDevice.h
    CameraDriverInterface.h
)

add_to_hal_sources( ${HDRS} )

install(FILES ${HDRS}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/HAL/Camera
)

add_subdirectory( Drivers )
