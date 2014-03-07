cmake_minimum_required( VERSION 2.8 )
project( gui-engine )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include
                     ${Pangolin_INCLUDE_DIRS}
                     ${SceneGraph_INCLUDE_DIRS}
                     ${HAL_INCLUDE_DIRS})

set( HDRS
    # include/gui-engine.h
     include/gl_tile.h
     include/pangolin_tile_view.h
     include/gl_ways.h
     include/GLSimpleObjects.h)
    # include/GLVehicleModel.h)

set( SRCS
   # src/gui-engine.cpp
    src/gl_tile.cpp
    src/pangolin_tile_view.cpp
    src/gl_ways.cpp)
   # src/GLVehicleModel.cpp )

add_library( gui-engine ${SRCS} ${HDRS})
target_link_libraries(gui-engine osm-engine ${Pangolin_LIBRARIES} ${SceneGraph_LIBRARIES} ${OpenCV2_LIBRARIES})


