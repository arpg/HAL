project( LogTools )
cmake_minimum_required(VERSION 2.8) 

# Add to module path, so we can find our cmake modules 
find_package( Pangolin REQUIRED )
find_package( SceneGraph REQUIRED )

set( _INCLUDE_DIRS 
        ${Pangolin_INCLUDE_DIRS}
        ${SceneGraph_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/..  
        )

set( _LINK_DIRS
        ${Pangolin_LINK_DIR}
        ${SceneGraph_LINK_DIR}
        )

include_directories( ${_INCLUDE_DIRS} )
link_directories( ${_LINK_DIRS} )

add_executable( beaver ViewerGui.h GLThumbnails.h Viewer.cpp )

target_link_libraries( beaver ${Pangolin_LIBRARIES} )
target_link_libraries( beaver ${SceneGraph_LIBRARIES} )
