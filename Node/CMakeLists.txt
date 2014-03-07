cmake_minimum_required(VERSION 2.8)

project(Node)
string( TOLOWER ${PROJECT_NAME} LIBRARY_NAME )

if(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "-DOS_OBJECT_USE_OBJC=0")
endif()

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -Wall -Wextra -Wno-unused-parameter")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wno-unused-parameter")

# Add to module path, so we can find our cmake modules
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH})

find_package(ZeroMQ QUIET)
find_package(Protobuf QUIET)
find_package(Boost COMPONENTS system thread QUIET)
find_package(MINIGLOG QUIET)

if(ZeroMQ_FOUND AND PROTOBUF_FOUND AND Boost_FOUND AND MINIGLOG_FOUND)
    set(should_build ON)
else()
    set(should_build OFF)
endif()

set(BUILD_NODE ${should_build} CACHE BOOL "To Build or not Build Node")

if( BUILD_NODE )
    if(NOT APPLE)
      find_package(DNSSD)
      find_package(Avahi)
    endif(NOT APPLE)

    if(DNSSD_FOUND OR APPLE)
      set(HAVE_DNSSD ON)
    endif()
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/NodeConfig.h.in
      ${CMAKE_CURRENT_BINARY_DIR}/NodeConfig.h)

    set(PROJ_INCLUDE_DIRS
      ${CMAKE_BINARY_DIR}
      ${CMAKE_SOURCE_DIR}/..
      ${ZeroMQ_INCLUDE_DIRS}
      ${Avahi_INCLUDE_DIRS}
      ${PROTOBUF_INCLUDE_DIRS}
      ${Node_SOURCE_DIR}/..
      ${Node_BINARY_DIR}
      ${Node_SOURCE_DIR}
      ${MINIGLOG_INCLUDE_DIRS}
      )

    set(PROJ_LIBRARIES
      ${PROTOBUF_LIBRARY}
      ${ZeroMQ_LIBRARIES}
      ${AVAHI_LIBRARIES}
      ${Boost_LIBRARIES}
      )

    if(DNSSD_FOUND)
      list(APPEND PROJ_INCLUDE_DIRS ${DNSSD_INCLUDE_DIRS})
      list(APPEND PROJ_LIBRARIES ${DNSSD_LIBRARIES})
    endif()

    include_directories(${PROJ_INCLUDE_DIRS})

    PROTOBUF_GENERATE_CPP(PROTO_HDRS PROTO_SRCS NodeMessages.proto)

    set(NODE_HDRS Node.h PrintMessage.h ZeroConf.h ${PROTO_HDRS})
    set(NODE_SRCS Node.cpp PrintMessage.cpp ZeroConf.cpp ${PROTO_SRCS})

    if(ANDROID)
      list(APPEND NODE_HDRS android-ifaddrs/ifaddrs.h)
      list(APPEND NODE_SRCS android-ifaddrs/ifaddrs.c)
      include_directories(SYSTEM android-ifaddrs)
    endif()

    add_library(${LIBRARY_NAME} ${NODE_HDRS} ${NODE_SRCS})
    target_link_libraries(${LIBRARY_NAME} ${PROJ_LIBRARIES})
    target_link_libraries(${LIBRARY_NAME} ${Boost_SYSTEM_LIBRARY} miniglog)

    # Create the NODEConfig.cmake file for the build tree.
    set(EXPORT_NODE_INC
      ${CMAKE_CURRENT_SOURCE_DIR}/..
      ${CMAKE_CURRENT_BINARY_DIR}
      ${PROTOBUF_INCLUDE_DIRS}
      ${ZeroMQ_INCLUDE_DIRS}
      )

    configure_file(${Node_SOURCE_DIR}/NodeConfig.cmake.in
      ${Node_BINARY_DIR}/NodeConfig.cmake @ONLY)

    set( EXPORT_LIB_INC_DIR "${LIB_INC_DIR}" )
    configure_file( "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in"
      "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY IMMEDIATE )

    set( EXPORT_LIB_INC_DIR "\${${PROJECT_NAME}_CMAKE_DIR}/${REL_INCLUDE_DIR}" )
    configure_file( "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in"
      "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY )


    # Add module to CMake package registery.
    get_target_property(miniglog_imported miniglog IMPORTED)
    if (NOT miniglog_imported)
      set(also_export_miniglog miniglog)
    endif()

    export(TARGETS ${LIBRARY_NAME} ${also_export_miniglog}
      FILE "${CMAKE_CURRENT_BINARY_DIR}/NodeTargets.cmake")

    option(EXPORT_NODE
      "Should the NODE package be exported for use by other software" ON)

    if(EXPORT_NODE)
      export(PACKAGE ${PROJECT_NAME})
    else()
      set(Node_DIR ${CMAKE_CURRENT_BINARY_DIR}) # For the examples
    endif()

    #######################################################
    ## Install headers / targets
    set( CMAKECONFIG_INSTALL_DIR "lib/cmake/${PROJECT_NAME}" )

    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/NodeConfig.h"
      DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${LIBRARY_NAME}
    )
    install(TARGETS ${LIBRARY_NAME}
      EXPORT "${PROJECT_NAME}Targets"
      RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
      LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
      ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    )

    #######################################################
    ## Install CMake config

    INSTALL(
	FILES "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
	      "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
	DESTINATION ${CMAKECONFIG_INSTALL_DIR} )

    install( EXPORT "${PROJECT_NAME}Targets" DESTINATION ${CMAKECONFIG_INSTALL_DIR} )

    add_subdirectory(Examples)
endif()
