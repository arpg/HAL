project(LexusLogger)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Wextra")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

find_package(Pangolin 0.1 QUIET)
find_package(HAL 0.1 QUIET)
find_package(SceneGraph QUIET)
find_package(OpenCV2 QUIET)
find_package(TinyXML2 QUIET)
find_package(MVL QUIET)


if(Pangolin_FOUND AND HAL_FOUND AND TinyXML2_FOUND AND MVL_FOUND)
    set(BUILD_LEXUSLOGGER OFF CACHE BOOL "Toggle building LexusLogger")
    
    if(BUILD_LEXUSLOGGER)
        add_executable( LexusLogger main.cpp )

        include_directories( ${HAL_INCLUDE_DIRS} )
        include_directories( ${Pangolin_INCLUDE_DIRS} )
        include_directories( ${SceneGraph_INCLUDE_DIRS} )
        target_link_libraries(LexusLogger ${HAL_LIBRARIES} dl mvl)
        target_link_libraries(LexusLogger ${Pangolin_LIBRARIES} )
        target_link_libraries(LexusLogger ${TINYXML2_LIBRARY})
        target_link_libraries(LexusLogger ${SceneGraph_LIBRARIES} )

        add_dependencies( LexusLogger hal )                                              
        add_dependencies( LexusLogger PbMsgs )   
    endif()
endif()
