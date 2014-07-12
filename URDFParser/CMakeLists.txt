cmake_minimum_required(VERSION 2.8)
project(URDFParser)
set(VERSION_MAJOR 0)
set(VERSION_MINOR 1)
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR})

include(def_library)
include(export_package)

string( TOLOWER ${PROJECT_NAME} LIBRARY_NAME )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall")
IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
ENDIF()

#######################################################
## Find required dependencies

find_package( Eigen3 REQUIRED )
find_package( Node REQUIRED )
find_package( MINIGLOG REQUIRED )
find_package( SceneGraph REQUIRED )
find_package( Pangolin REQUIRED )
find_package( Calibu 0.1 REQUIRED )
find_package( PbMsgs REQUIRED )
find_package( TinyXML2 REQUIRED )
find_package( OpenCV2 )

list( APPEND LIB_INC_DIR "${CMAKE_CURRENT_BINARY_DIR}/..;${CMAKE_CURRENT_SOURCE_DIR}/.." )
list( APPEND USER_INC   ${CMAKE_CURRENT_BINARY_DIR} )
list( APPEND USER_INC   ${CMAKE_CURRENT_SOURCE_DIR} )

list( APPEND USER_INC   ${EIGEN3_INCLUDE_DIR} )
list( APPEND USER_INC   ${PROTOBUF_INCLUDE_DIR} )
list( APPEND USER_INC   ${MINIGLOG_INCLUDE_DIRS} )
list( APPEND USER_INC   ${Node_INCLUDE_DIRS})
list( APPEND USER_INC   ${SceneGraph_INCLUDE_DIRS})
list( APPEND USER_INC   ${Pangolin_INCLUDE_DIRS})
list( APPEND USER_INC   ${Calibu_INCLUDE_DIRS})
list( APPEND USER_INC   ${PbMsgs_INCLUDE_DIRS})
list( APPEND USER_INC   ${TinyXML2_INCLUDE_DIRS})

list( APPEND LINK_LIBS  ${PROTOBUF_LIBRARIES} )
list( APPEND LINK_LIBS  ${MINIGLOG_LIBRARIES})
list( APPEND LINK_LIBS  ${Node_LIBRARIES})
list( APPEND LINK_LIBS  ${SceneGraph_LIBRARIES})
list( APPEND LINK_LIBS  ${Pangolin_LIBRARIES})
list( APPEND LINK_LIBS  ${Calibu_LIBRARIES})
list( APPEND LINK_LIBS  ${PbMsgs_LIBRARIES})
list( APPEND LINK_LIBS  ${TinyXML2_LIBRARIES})

include_directories( ${LIB_INC_DIR} )
include_directories( ${USER_INC} )

if(ANDROID)
  # Override to use hosts protoc compiler
  unset(PROTOBUF_PROTOC_EXECUTABLE CACHE)
  find_host_package(Protobuf REQUIRED)
endif()

#######################################################
## Sources

list(APPEND SIMDEVICES_SRCS
  SimDevices/SimDevices.cpp
)

list(APPEND URDFPARSER_SRCS
  URDFParser.cpp
)

list(APPEND SRCS
  ${URDFPARSER_SRCS}
  ${SIMDEVICES_SRCS})

#######################################################
## Headers

list(APPEND SIMDEVICES_HDRS
  SimDevices/SimDeviceInfo.h
  SimDevices/SimDevices.h
)

list(APPEND BULLETSTRUCT_HDRS
  BulletStructs/ModelNode.h
  BulletStructs/Shape.h
  BulletStructs/Constraint.h
  BulletStructs/VehicleEnums.h
  BulletStructs/SimRaycastVehicle.h
  )

list(APPEND SIMDEVICES_CONTROLLER_HDRS
  SimDevices/Controller/Controller.h
  SimDevices/Controller/CarController.h
  SimDevices/Controller/PIDControl.h
  SimDevices/Controller/SimpleAIController.h
  SimDevices/Controller/PIDController.h
  SimDevices/Controller/SimpleController.h
  SimDevices/Controller/PoseController.h
)

list(APPEND SIMDEVICES_SENSOR_HDRS
  SimDevices/Sensor/SimGPS.h
  SimDevices/Sensor/SimLaser3D.h
  SimDevices/Sensor/SimVicon.h
  SimDevices/Sensor/SimCamera.h
  SimDevices/Sensor/SimLaser2D.h
  SimDevices/Sensor/SimOdometry.h
)

list(APPEND SIMROBOTS_HDRS
  SimRobots/SimRobot.h
  SimRobots/SimWorld.h
)

list(APPEND URDFPARSER_HDRS
  URDFParser.h
  TinyXMLTool.h
  GenURIFromURDF.h
  EigenHelpers.h
)

list(APPEND HRDS
  ${SIMDEVICES_HDRS}         ${SIMDEVICES_CONTROLLER_HDRS}
  ${SIMDEVICES_SENSOR_HDRS}  ${SIMROBOTS_HDRS}
  ${URDFPARSER_HDRS} 
  )

#######################################################
## Define the library

def_library(${LIBRARY_NAME}
  SOURCES ${SRCS} ${HDRS}
  LINK_LIBS ${LINK_LIBS}
  PACKAGE URDFParser
  )

export_package(${PROJECT_NAME}
  TARGETS ${LIBRARY_NAME}
  VERSION ${VERSION}
  LIBRARY ${LIBRARY_NAME}
  LIBRARIES ${LIBRARY_NAME}
  INCLUDE_DIRS ${LIB_INC_DIR} ${USER_INC}
  )

#######################################################
## Create configure file for inclusion in library

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/config.h"
  )

#######################################################
## Install headers / targets

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config.h"
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
  )
install(FILES ${HDRS}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
  )
