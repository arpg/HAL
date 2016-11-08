set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )

find_package( IOKit QUIET )
find_package( CoreFoundation QUIET)

## There is a problem in add_to_hal_libraries. if I link sys_libs inside if(build_gamepad) then
## I will have linking errors when compiling the code. this happens when BUILD_GAMEPAD is OFF by default.
set( SYS_LIBS ${IOKit_LIBRARIES}	${COREFOUNDATION_LIBRARY})
add_to_hal_libraries( ${SYS_LIBS} )
add_to_hal_include_dirs( ${IOKit_INCLUDE_DIRS} )

set( BUILD_GAMEPAD OFF CACHE BOOL force)
if (BUILD_GAMEPAD)
	if(UNIX)
			set( GSDK_CPP GSDK/Gamepad_private.c )
			set( GSDK_H GSDK/Gamepad_private.h GSDK/Gamepad.h )
			if(APPLE)
					if( COREFOUNDATION_FOUND AND IOKit_FOUND )
							set( GSDK_CPP ${GSDK_CPP} GSDK/Gamepad_macosx.c )
					endif( COREFOUNDATION_FOUND AND IOKit_FOUND )
			else(APPLE)
					set( GSDK_CPP ${GSDK_CPP} GSDK/Gamepad_linux.c )
			endif(APPLE)
	endif(UNIX)
	add_to_hal_sources( GamepadMultiDeviceDriver.h ${GSDK_H} GamepadMultiDeviceDriver.cpp GamepadMultiDeviceFactory.cpp ${GSDK_CPP} )
endif( BUILD_GAMEPAD )

