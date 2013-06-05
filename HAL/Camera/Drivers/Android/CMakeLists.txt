if( ANDROID )
    set( Build_Android true )
elseif()
    set( Build_Android false CACHE BOOL force )
endif()

set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )
find_package( Raisin QUIET )

if( Build_Android AND RAISIN_FOUND)
    # TODO make a find script for these
    add_to_hal_include_dirs( ${Raisin_INCLUDE_DIRS} )
    add_to_hal_libraries( ${Raisin_LIBRARIES} )
    add_to_hal_sources(
        AndroidDriver.h AndroidDriver.cpp CameraListener.h CameraListener.cpp AndroidFactory.cpp
    )
endif()
