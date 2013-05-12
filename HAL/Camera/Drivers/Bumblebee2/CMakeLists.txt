set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )

find_package( DC1394_2 QUIET )
if( DC1394_2_FOUND )
    add_definitions( -DUSE_BUMBLEBEE_2 )
    
    # 1) Build the library:
    set( BB2_DRIVER_SOURCES Bumblebee2Driver.cpp )
    set( BB2_DRIVER_HEADERS Bumblebee2Driver.h )
    add_library( Bumblebee2Driver ${BB2_DRIVER_SOURCES} ${BB2_DRIVER_HEADERS} )
    
    # 2) Tell HAL about this library:
    add_to_hal_include_dirs( ${DC1394_2_INCLUDE_DIR} )
    add_to_hal_libraries( Bumblebee2Driver )
    
    list( APPEND HAL_LIBRARIES ${DC1394_2_LIBRARIES} )
    set( HAL_LIBRARIES ${HAL_LIBRARIES} CACHE INTERNAL "" FORCE )
    
    # 3) And make sure we are able to install it correctly:
    install( TARGETS Bumblebee2Driver
      RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
      LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
      ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    )
    
    install( FILES ${BB2_DRIVER_HEADERS}
             DESTINATION
             ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/Bumblebee2 )
endif()

