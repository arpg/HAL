# build the library
set( MIPSDK_CPP MIPSDK/mip_sdk_user_functions.cpp MIPSDK/byteswap_utilities.cpp MIPSDK/mip_sdk_base.cpp MIPSDK/mip_sdk_gps.cpp MIPSDK/mip.cpp MIPSDK/mip_sdk_inteface.cpp MIPSDK/ring_buffer.cpp MIPSDK/mip_sdk_3dm.cpp MIPSDK/mip_sdk_nav.cpp MIPSDK/mip_sdk_ahrs.cpp MIPSDK/mip_sdk_system.cpp )
set( MIPSDK_H MIPSDK/mip_sdk_user_functions.h MIPSDK/byteswap_utilities.h MIPSDK/mip.h MIPSDK/mip_sdk_config.h MIPSDK/mip_sdk_nav.h MIPSDK/mainpage.h MIPSDK/mip_sdk_3dm.h MIPSDK/mip_sdk_gps.h MIPSDK/mip_sdk_system.h MIPSDK/mip_gx3_35.h MIPSDK/mip_sdk_ahrs.h MIPSDK/mip_sdk.h MIPSDK/mip_types.h MIPSDK/mip_gx3_45.h MIPSDK/mip_sdk_base.h MIPSDK/mip_sdk_interface.h MIPSDK/ring_buffer.h )

set( BUILD_MicroStrain true CACHE BOOL force )

if( BUILD_MicroStrain )
    add_to_hal_sources( MicroStrainDriver.h ${MIPSDK_H} MicroStrainDriver.cpp MicroStrainFactory.cpp ${MIPSDK_CPP} )
endif()
