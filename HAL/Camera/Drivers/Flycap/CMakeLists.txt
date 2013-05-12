set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )

find_package( FLYCAP QUIET )
if( FLYCAP_FOUND )
    add_definitions( -DUSE_FLYCAP )

    # 1) Build the library:
    set( FLY_DRIVER_SOURCES FlycapDriver.cpp )
    set( FLY_DRIVER_HEADERS FlycapDriver.h )
    add_library( FlycapDriver ${FLY_DRIVER_SOURCES} ${FLY_DRIVER_HEADERS} )
    
    # 2) Tell HAL about this library:
    add_to_hal_include_dirs( ${FLYCAP_INCLUDE_DIR} )
    add_to_hal_libraries( FlycapDriver )
    
    list( APPEND HAL_LIBRARIES ${FLYCAP_LIBRARIES} )
    set( HAL_LIBRARIES ${HAL_LIBRARIES} CACHE INTERNAL "" FORCE )
    
    # 3) And make sure we are able to install it correctly:
    install( TARGETS FlycapDriver
      RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
      LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
      ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    )
    
    install( FILES ${FLY_DRIVER_HEADERS}
             DESTINATION
             ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/Flycap )
endif()

