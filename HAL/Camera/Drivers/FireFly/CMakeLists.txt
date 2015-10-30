if(0)
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )

find_package( DC1394_2 QUIET )
if( DC1394_2_FOUND )
    add_definitions( -DUSE_FIREFLY )

    message( STATUS "HAL: building 'FireFly' camera driver.")

    # 1) Build the library:
    set( FF_DRIVER_SOURCES FireFlyDriver.cpp )
    set( FF_DRIVER_HEADERS FireFlyDriver.h )
    add_library( FireFlyDriver ${FF_DRIVER_SOURCES} ${FF_DRIVER_HEADERS} )
    
    # 2) Tell HAL about this library:
    add_to_hal_include_dirs( ${DC1394_2_INCLUDE_DIR} )
    add_to_hal_libraries( FireFlyDriver )
    
    list( APPEND HAL_LIBRARIES ${DC1394_2_LIBRARIES} )
    set( HAL_LIBRARIES ${HAL_LIBRARIES} CACHE INTERNAL "" FORCE )
    
    # 3) And make sure we are able to install it correctly:
    install( TARGETS FireFlyDriver
      RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
      LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
      ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    )
    
    install( FILES ${FF_DRIVER_HEADERS}
             DESTINATION
             ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/FireFly )
endif()
endif()
