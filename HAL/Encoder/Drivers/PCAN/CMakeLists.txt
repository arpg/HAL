set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )

find_package( PCAN QUIET )

if( PCAN_FOUND )
    set( BUILD_PCAN false CACHE BOOL force )
    if( BUILD_PCAN )
      add_to_hal_include_dirs( ${PCAN_INCLUDE_DIRS} )
      add_to_hal_libraries( ${PCAN_LIBRARIES} )
      add_to_hal_sources( PCANEncoderDriver.h PCANEncoderDriver.cpp PCANEncoderFactory.cpp PCANListener.h)
    endif()
endif()
