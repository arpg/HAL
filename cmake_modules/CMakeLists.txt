cmake_minimum_required( VERSION 2.8 )

project( Mochaccino )

# Add generic libs/headers we often use
include( Common.cmake )

#the boost libraries that we need
set(BOOST_LIBS thread date_time system)

# Add to module path, so we can find our cmake modules
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )

# things we need
#find_package(LibAO REQUIRED)
find_package( MVL REQUIRED )
find_package( CVARS REQUIRED )
#find_package( ALSA REQUIRED )
find_package( Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
find_package( Eigen3 REQUIRED )
find_package( OpenGL REQUIRED )
find_package( FLTK REQUIRED )
find_package( CVARS REQUIRED )
find_package( CarPlanner REQUIRED )
find_package( ZeroMQ REQUIRED )
find_package( Protobuf REQUIRED )

# include dirs
include_directories( ${MVL_INCLUDE_DIRS} )
include_directories( ${CVARS_INCLUDE_DIR} )
include_directories( ${BOOST_INCLUDE_DIR} )
#include_directories( ${ALSA_INCLUDE_DIR} )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories( ${FLTK_INCLUDE_DIR} )
include_directories( ${CVARS_INCLUDE_DIR} )
include_directories( ${CAR_PLANNER_INCLUDE_DIRS} )
include_directories( ${ZeroMQ_INCLUDE_DIRS} )
include_directories( ${PROTOBUF_INCLUDE_DIRS} )

add_subdirectory( Physics )

#generate the protobuf cc and header files and make sure the binary
#directories are included (as they'll end up there
include_directories( ${CMAKE_BINARY_DIR} )
PROTOBUF_GENERATE_CPP( PROTO_SRCS PROTO_HDRS Messages.proto )

# Mochaccino Files
set( HDRS AudioBuffer.h MochaException.h Ppm.h MochaConfig.h Vicon.h Observer.h Controller.h GLConsole.h Lqr.h Common.h MochaConfig.h AirController.h TrajectoryTracker.h Node.h Messages.pb.h Pid.h)
set( SRCS Mochaccino.cpp AudioBuffer.cpp MochaException.cpp Ppm.cpp Vicon.cpp Observer.cpp Controller.cpp Lqr.cpp AirController.cpp TrajectoryTracker.cpp Messages.pb.cc Pid.cpp)



#library dirs
link_directories( ${Boost_LIBRARY_DIRS} )
link_directories( ${CAR_PLANNER_LIBRARY_DIRS} )

add_executable( Mochaccino ${HDRS} ${SRCS})
target_link_libraries( Mochaccino vrpn vrpnserver vrpn_atmel vrpn_timecode_generator )

#target_link_libraries( Mochaccino ${MVL_LIBRARIES} )
#target_link_libraries( Mochaccino pangolin )
target_link_libraries( Mochaccino ao )
target_link_libraries( Mochaccino ${CAR_PLANNER_LIBRARIES} )
target_link_libraries( Mochaccino ${ZeroMQ_LIBRARIES} )
target_link_libraries( Mochaccino ${PROTOBUF_LIBRARIES} )
target_link_libraries( Mochaccino ${CVARS_LIBRARIES} )
#target_link_libraries( Mochaccino ${ALSA_LIBRARIES} )	# ALSA libraries for the PPM system
target_link_libraries( Mochaccino ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ) #link in the boost libraries we want
target_link_libraries( Mochaccino ${OPENGL_LIBRARIES} ${CVARS_LIBRARIES} ${FLTK_LIBRARIES} )
