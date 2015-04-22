project( LogViewer )
cmake_minimum_required(VERSION 2.8)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall  -Werror")

# Add to module path, so we can find our cmake modules
find_package( Pangolin 0.1 QUIET )
find_package( SceneGraph 0.1 QUIET )

if( Pangolin_FOUND AND SceneGraph_FOUND )
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

    add_executable( LogViewer ViewerGui.h GLThumbnailView.h Viewer.cpp )

    target_link_libraries( LogViewer ${Pangolin_LIBRARIES} )
    target_link_libraries( LogViewer ${SceneGraph_LIBRARIES} )

    add_dependencies( LogViewer hal )

endif()
