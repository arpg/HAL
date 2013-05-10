# 1) Build the library
set( MIPSDK_CPP MIPSDK/mip_sdk_user_functions.cpp MIPSDK/byteswap_utilities.cpp MIPSDK/mip_sdk_base.cpp MIPSDK/mip_sdk_gps.cpp MIPSDK/mip.cpp MIPSDK/mip_sdk_inteface.cpp MIPSDK/ring_buffer.cpp MIPSDK/mip_sdk_3dm.cpp MIPSDK/mip_sdk_nav.cpp MIPSDK/mip_sdk_ahrs.cpp MIPSDK/mip_sdk_system.cpp )
set( MIPSDK_H MIPSDK/mip_sdk_user_functions.h MIPSDK/byteswap_utilities.h MIPSDK/mip.h MIPSDK/mip_sdk_config.h MIPSDK/mip_sdk_nav.h MIPSDK/mainpage.h MIPSDK/mip_sdk_3dm.h MIPSDK/mip_sdk_gps.h MIPSDK/mip_sdk_system.h MIPSDK/mip_gx3_35.h MIPSDK/mip_sdk_ahrs.h MIPSDK/mip_sdk.h MIPSDK/mip_types.h MIPSDK/mip_gx3_45.h MIPSDK/mip_sdk_base.h MIPSDK/mip_sdk_interface.h MIPSDK/ring_buffer.h )

# Ignore warnings generated from third party includes
if(CMAKE_COMPILER_IS_GNUCXX)
    set_source_files_properties( ${MIPSDK_CPP} PROPERTIES COMPILE_FLAGS " -w " )
endif(CMAKE_COMPILER_IS_GNUCXX)

set( MICROSTRAIN_DRIVER_SOURCES MicroStrainDriver.cpp ${MIPSDK_CPP} )
set( MICROSTRAIN_DRIVER_HEADERS MicroStrainDriver.h ${MIPSDK_H} )
add_library( MicroStrainDriver ${MICROSTRAIN_DRIVER_SOURCES} ${MICROSTRAIN_DRIVER_HEADERS} )


# 2) Tell HAL about this library:
add_to_hal_include_dirs( ${CMAKE_CURRENT_SOURCE_DIR} )
add_to_hal_libraries( MicroStrainDriver )
add_to_hal_link_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# 3) And make sure we are able to install it correctly:
install( TARGETS MicroStrainDriver
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

install( FILES ${MICROSTRAIN_DRIVER_HEADERS}
         DESTINATION
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/IMU/Drivers/MicroStrain )
