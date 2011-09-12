
#add_library( camlib CameraDriverRegistery.cpp )
#add_library( bb2lib Bumblebee2Driver.cpp )
#add_library( frlib FileReaderDriver.cpp )
#SET( RPG_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} )
#SET( RPG_LIBRARIES camlib bb2lib frlib )

subdir_depends( Test Drivers )
add_subdirectory( Drivers )
add_subdirectory( Test )

#add_executable( Test Test.cpp )
#target_link_libraries( Test camlib bb2lib frlib )
