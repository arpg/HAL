set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )

find_package( Node QUIET )

if( Node_FOUND )
    set( BUILD_NodeIMU true CACHE BOOL force )
    if( BUILD_NodeIMU )
        add_to_hal_include_dirs( ${Node_INCLUDE_DIRS} )
        add_to_hal_libraries( ${Node_LIBRARIES} )
        add_to_hal_sources( NodeIMUDriver.h NodeIMUDriver.cpp NodeIMUFactory.cpp )
    endif()
endif()
