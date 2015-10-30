# Build AlliedVision driver (you also need to make sure to edit DriverList.h)
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )
find_package( PVAPI QUIET )
if( PVAPI_FOUND )
    add_definitions( -DUSE_ALLIEDVISION )
    
    message( STATUS "HAL: building 'AlliedVision' camera driver.")

    # 1) Build the library:
    set( AV_DRIVER_SOURCES AlliedVisionDriver.cpp )
    set( AV_DRIVER_HEADERS AlliedVisionDriver.h )
    add_library( AlliedVisionDriver ${AV_DRIVER_SOURCES} ${AV_DRIVER_HEADERS} )
    
    include_directories( AlliedVisionDriver ${PVAPI_INCLUDE_DIR} )
    target_link_libraries(AlliedVisionDriver ${PVAPI_LIBRARIES} )
    
    # 2) Tell HAL about this library:
    add_to_hal_libraries( AlliedVisionDriver )
    
    list( APPEND HAL_LIBRARIES "${PVAPI_LIBRARIES}" )
    set( HAL_LIBRARIES ${HAL_LIBRARIES} CACHE INTERNAL "" FORCE )
    
    # 3) And make sure we are able to install it correctly:
    install( TARGETS AlliedVisionDriver
      RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
      LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
      ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    )
endif()
