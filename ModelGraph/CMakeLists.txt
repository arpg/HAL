project( ModelGraph )
    cmake_minimum_required(VERSION 2.8) 
         
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall  -Werror -DBOOST_NO_CXX11_NUMERIC_LIMITS")

#IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
#    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
#ENDIF()

# Add to module path, so we can find our cmake modules 
    set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )

    find_package( Eigen3 REQUIRED )
    find_package( Protobuf REQUIRED )
    find_package( Sophus REQUIRED )

    set( _INCLUDE_DIRS 
                    ${Sophus_INCLUDE_DIR}
                    ${EIGEN3_INCLUDE_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}/..
        )

    include_directories( ${_INCLUDE_DIRS} )

# expose these for third parties
    set( MODELGRAPH_INCLUDE_DIRS ${_INCLUDE_DIRS}  CACHE INTERNAL "" FORCE )

    get_target_property( _LIBRARY ModelGraph LOCATION )
    list( INSERT MODELGRAPH_LIBRARIES 0 ${_LIBRARY} )
     
# Create the *Config.cmake file for the build tree.
    configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/ModelGraphConfig.cmake.in
                        ${CMAKE_CURRENT_BINARY_DIR}/ModelGraphConfig.cmake @ONLY IMMEDIATE )

# Add module to CMake package registery.
    export( PACKAGE ModelGraph )


