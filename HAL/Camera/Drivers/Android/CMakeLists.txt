if(ANDROID)
  set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH})

  find_package(AndroidKernel QUIET)
  find_package(GLUES QUIET)

  if(AndroidKernel_FOUND AND GLUES_FOUND)
    set(BUILD_Android ON CACHE BOOL "Build the generic Android camera driver")
  elseif()
    set(BUILD_Android OFF CACHE BOOL "Build the generic Android camera driver")
  endif()

  if(BUILD_Android)
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
