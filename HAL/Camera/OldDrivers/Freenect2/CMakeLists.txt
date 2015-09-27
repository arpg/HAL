find_package( freenect2 QUIET )

set( BUILD_Freenect2 "${freenect2_FOUND}" CACHE BOOL
  "Build Freenect2 driver for Kinect2" )

set(freenect2_DEFINITIONS "-DLIBFREENECT2_THREADING_TINYTHREAD")
##temp fix for OpenKinect/libfreenect2 issue #217
#set(freenect2_INCLUDE_DIRS ${freenect2_INCLUDE_DIR} ${freenect2_INCLUDE_DIR}/libfreenect2/tinythread)
# Whoa, this should not be here -- at least move it inside the condtional bellow

if( BUILD_Freenect2 )

  set(freenect2_DEFINITIONS "-DLIBFREENECT2_THREADING_TINYTHREAD")
  #temp fix for OpenKinect/libfreenect2 issue #217
  set(freenect2_INCLUDE_DIRS ${freenect2_INCLUDE_DIR} ${freenect2_INCLUDE_DIR}/libfreenect2/tinythread)

  if( freenect2_FOUND )

    message( STATUS "HAL: building 'Freenect2' camera driver.")

    add_to_hal_include_dirs( ${freenect2_INCLUDE_DIRS} )
    add_to_hal_libraries( ${freenect2_LIBRARY} )
    add_to_hal_sources( Freenect2Driver.h Freenect2Driver.cpp
      Freenect2Factory.cpp depth_registration.h depth_registration.cpp depth_registration_cpu.h depth_registration_cpu.cpp)
    hal_set_compile_flags( ${CMAKE_CURRENT_SOURCE_DIR}/Freenect2Driver.cpp ${freenect2_DEFINITIONS} )
	  hal_set_compile_flags( ${CMAKE_CURRENT_SOURCE_DIR}/Freenect2Factory.cpp ${freenect2_DEFINITIONS} )
  else()
	  message("libfreenect2 not found. Freenect2 driver cannot be built")
  endif()
endif()

