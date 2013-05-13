set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )

find_package( OpenNI QUIET )

set( Build_OpenNI true CACHE BOOL force )

if( OPENNI_FOUND )
if( Build_OpenNI )
    add_to_hal_libraries( ${OpenNI_LIBRARIES} )
    add_to_hal_sources( OpenNIDriver.h OpenNIDriver.cpp OpenNIFactory.cpp )
endif()
endif()
