set(HDRS
    PosysDevice.h
    PosysDriverInterface.h
)

add_to_hal_sources( ${HDRS} )

install(FILES ${HDRS}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/HAL/Posys
)

add_subdirectory( Drivers )
