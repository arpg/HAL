set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )

find_package( PHIDGETS QUIET )

set( BUILD_Phidgets false CACHE BOOL force )

message( STATUS "Phidgets INCS: ${PHIDGETS_INCLUDE_DIR}" )
message( STATUS "Phidgets LIBS: ${PHIDGETS_LIBRARIES}" )

if( PHIDGETS_FOUND AND BUILD_Phidgets )
    add_to_hal_include_dirs( ${PHIDGETS_INCLUDE_DIR} )
    add_to_hal_libraries( ${PHIDGETS_LIBRARIES} )
    add_to_hal_sources( PhidgetsDriver.h PhidgetsDriver.cpp PhidgetsFactory.cpp )
endif()
