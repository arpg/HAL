if(ANDROID)
  set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH})

  find_package(AndroidKernel QUIET)
  find_package(GLUES QUIET)

  set(BUILD_KitKat OFF CACHE BOOL "Build the KitKat camera driver")
  if(BUILD_KitKat)

    message( STATUS "HAL: building 'KitKat' camera driver.")

    add_to_hal_include_dirs(${AndroidKernel_INCLUDE_DIRS})
    add_to_hal_libraries(${AndroidKernel_LIBRARIES})

    add_to_hal_libraries(${GLUES_LIBRARY} EGL GLESv2 GLESv1_CM
      c m dl utils camera_client binder log ui camera_metadata)

    add_to_hal_sources(KitKatFactory.cpp Camera.cpp KitKatDriver.cpp)
    hal_set_compile_flags(${CMAKE_CURRENT_SOURCE_DIR}/KitKatDriver.cpp "-fpermissive")
    hal_set_compile_flags(${CMAKE_CURRENT_SOURCE_DIR}/Camera.cpp "-fpermissive -fno-rtti")
    hal_set_compile_flags(${CMAKE_CURRENT_SOURCE_DIR}/KitKatFactory.cpp "-fpermissive")
  endif()

endif()
