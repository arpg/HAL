
  set( BUILD_Undistort true CACHE BOOL force )

  if( BUILD_Undistort )
      find_package( Calibu 0.1 QUIET )
      if(Calibu_FOUND)
        message( STATUS "HAL: building 'Undistort' camera driver.")
	add_to_hal_libraries( ${Calibu_LIBRARIES} )
      	add_to_hal_include_dirs( ${Calibu_INCLUDE_DIRS} )
      	add_to_hal_sources(
          UndistortDriver.h UndistortDriver.cpp UndistortFactory.cpp
      	  )
      endif()
endif()
