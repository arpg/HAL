project(Gamepad)
cmake_minimum_required(VERSION 2.8)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

set( GAMEPAD_HDRS   Gamepad.h
                    Gamepad_private.h
                 )

set( GAMEPAD_SRC    Gamepad_private.c
                 )

# test platform type
if(UNIX)
    if(APPLE)
				find_package( IOKit REQUIRED )
				include_directories( ${IOKit_INCLUDE_DIRS} )
				find_library(COREFOUNDATION_FRAMEWORK CoreFoundation)
				message("Finding library CoreFoundation: ${COREFOUNDATION_FRAMEWORK}")
				set( GAMEPAD_SRC ${GAMEPAD_SRC} Gamepad_macosx.c )
        set( GAMEPAD_LIBRARIES ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
    else()
        set( GAMEPAD_SRC ${GAMEPAD_SRC} Gamepad_linux.c )
    endif()
endif()

add_library(Gamepad ${GAMEPAD_HDRS} ${GAMEPAD_SRC} )
target_link_libraries(Gamepad ${GAMEPAD_LIBRARIES} ${IOKit_LIBRARIES} ${COREFOUNDATION_FRAMEWORK})
