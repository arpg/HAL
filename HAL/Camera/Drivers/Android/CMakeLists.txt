if( ANDROID )
    set( Build_Android true )
elseif()
    set( Build_Android false CACHE BOOL force )
endif()

if( Build_Android)
    set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )
    
    find_package( AndroidKernel REQUIRED )
    add_to_hal_include_dirs( ${AndroidKernel_INCLUDE_DIRS} )
    add_to_hal_libraries( ${AndroidKernel_LIBRARIES} )

    find_package(GLUES REQUIRED)
    add_to_hal_libraries( ${GLUES_LIBRARY} EGL GLESv2 GLESv1_CM )

    add_to_hal_sources(
        AndroidDriver.h AndroidDriver.cpp
        CameraListener.h CameraListener.cpp
        AndroidFactory.cpp
    )
endif()
