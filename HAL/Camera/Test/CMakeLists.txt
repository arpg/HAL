include_directories( ${RPG_INCLUDE_DIRS} )
add_executable( TestCameraDevice TestCameraDevice.cpp )
target_link_libraries( TestCameraDevice ${RPG_LIBRARIES} )
# have to add this here for make -j
add_dependencies( TestCameraDevice CameraDriverRegistery )
add_dependencies( TestCameraDevice FileReaderDriver )
