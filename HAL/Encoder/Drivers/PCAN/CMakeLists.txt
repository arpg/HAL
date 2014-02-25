set( BUILD_PCAN false CACHE BOOL force )
if( BUILD_PCAN )
    add_to_hal_sources( PCANEncoderDriver.h PCANEncoderDriver.cpp PCANEncoderFactory.cpp PCANListener.h)
endif()
