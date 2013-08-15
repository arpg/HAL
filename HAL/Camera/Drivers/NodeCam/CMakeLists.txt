if(0)
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )

find_package( ZeroMQ QUIET )

if( ZeroMQ_FOUND )

    set( BUILD_NodeCam true CACHE BOOL force )
    if( BUILD_NodeCam )
        add_to_hal_include_dirs( ${ZeroMQ_INCLUDE_DIRS} )
        add_to_hal_libraries( ${ZeroMQ_LIBRARIES} )
        add_to_hal_sources( NodeCamDriver.h NodeCamDriver.cpp NodeCamFactory.cpp )
    endif()
endif()
endif()
