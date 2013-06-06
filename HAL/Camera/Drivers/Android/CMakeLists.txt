if( ANDROID )
    set( Build_Android true )
elseif()
    set( Build_Android false CACHE BOOL force )
endif()

if( Build_Android )
    set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )

    add_to_hal_sources(
        AndroidDriver.h AndroidDriver.cpp
        CameraListener.h CameraListener.cpp
        AndroidFactory.cpp
    )
    set_property( GLOBAL APPEND PROPERTY P_LIBRARIES -llog )
endif()
