set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )

find_package( OpenNI2 QUIET )

message( status "OPENNI2_LIBRARIES ${OPENNI2_LIBRARIES}")

if( OPENNI2_FOUND )
    set( BUILD_OpenNI2 true CACHE BOOL force )
    if( BUILD_OpenNI2 )
        add_to_hal_include_dirs( ${OPENNI2_INCLUDE_DIRS} )
        add_to_hal_libraries( ${OPENNI2_LIBRARY} )
        add_to_hal_sources( OpenNI2Driver.h OpenNI2Driver.cpp OpenNI2Factory.cpp )
    endif()
endif()
