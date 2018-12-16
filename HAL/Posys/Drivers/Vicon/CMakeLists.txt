set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )

find_package( VRPN QUIET )

if( VRPN_FOUND )
    set( BUILD_Vicon true CACHE BOOL force )

    if( BUILD_Vicon )
        add_to_hal_include_dirs( ${VRPN_INCLUDE_DIR} )
        add_to_hal_libraries( ${VRPN_LIBRARIES} )
        add_to_hal_sources( ViconDriver.h ViconDriver.cpp ViconFactory.cpp )
    endif()
endif()
