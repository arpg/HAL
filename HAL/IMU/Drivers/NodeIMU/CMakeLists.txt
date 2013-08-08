set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )

find_package( ZeroMQ QUIET )

set( BUILD_NodeIMU true CACHE BOOL force )

if( ZeroMQ_FOUND AND BUILD_NodeIMU )
    add_to_hal_include_dirs( ${ZeroMQ_INCLUDE_DIRS} )
    add_to_hal_libraries( ${ZeroMQ_LIBRARIES} )
    add_to_hal_sources( NodeIMUDriver.h NodeIMUDriver.cpp NodeIMUFactory.cpp )
endif()
