set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )

find_package( VRPN QUIET )

set( BUILD_Vicon false CACHE BOOL force )

message( STATUS "VRPN INCS: ${VRPN_INCLUDE_DIR}" )
message( STATUS "VRPN LIBS: ${VRPN_LIBRARIES}" )

if( VRPN_FOUND AND BUILD_Vicon )
    add_to_hal_include_dirs( ${VRPN_INCLUDE_DIR} )
    add_to_hal_libraries( ${VRPN_LIBRARIES} )
    add_to_hal_sources( ViconDriver.h ViconDriver.cpp ViconFactory.cpp )
endif()
