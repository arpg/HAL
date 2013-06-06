if( ANDROID )
    set( Build_Android true )
elseif()
    set( Build_Android false CACHE BOOL force )
endif()

set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )
find_package( AndroidKernel QUIET )

if( Build_Android AND AndroidKernel_FOUND)
    add_to_hal_include_dirs( ${AndroidKernel_INCLUDE_DIRS} )
    add_to_hal_sources(
        AndroidDriver.h AndroidDriver.cpp
        CameraListener.h CameraListener.cpp
        AndroidFactory.cpp
    )
    set_property( GLOBAL APPEND PROPERTY P_LIBRARIES -llog )
endif()
