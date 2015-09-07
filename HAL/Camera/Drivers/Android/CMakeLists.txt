if(ANDROID)
  set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH})

  find_package(AndroidKernel QUIET)
  find_package(GLUES QUIET)

  set(BUILD_Android OFF CACHE BOOL "Build the generic Android camera driver")

  if(BUILD_Android)

    message( STATUS "HAL: building 'Android' camera driver.")

    add_to_hal_include_dirs( ${AndroidKernel_INCLUDE_DIRS} )
    add_to_hal_libraries( ${AndroidKernel_LIBRARIES} )

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

endif()
