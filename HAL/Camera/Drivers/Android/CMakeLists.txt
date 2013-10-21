if( ANDROID )
    set( BUILD_Android true CACHE BOOL force )
elseif()
    set( BUILD_Android false )
endif()

if( BUILD_Android )
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
    hal_set_compile_flags(${CMAKE_CURRENT_SOURCE_DIR}/CameraListener.cpp "-fno-rtti -fpermissive")
    hal_set_compile_flags(${CMAKE_CURRENT_SOURCE_DIR}/AndroidDriver.cpp "-fpermissive")
    hal_set_compile_flags(${CMAKE_CURRENT_SOURCE_DIR}/AndroidFactory.cpp "-fpermissive")
endif()
