set(HDRS
    LIDARDevice.h
    LIDARDriverInterface.h
)

add_to_hal_sources( ${HDRS} )

install(FILES ${HDRS}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/HAL/LIDAR
)

add_subdirectory( Drivers )
