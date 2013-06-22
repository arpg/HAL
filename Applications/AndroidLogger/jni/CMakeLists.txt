set( SRCS main.cpp )

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_library(${PROJECT_NAME} SHARED ${SRCS} )

# NDK libraries
target_link_libraries(${PROJECT_NAME} log android )

# .so libraries pulled from machine folder /System and placed on path
target_link_libraries(${PROJECT_NAME} cameraservice camera_client utils cutils gui binder)

# Android Kernel headers
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/kernel/frameworks/av/include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/kernel/frameworks/native/include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/kernel/hardware/libhardware/include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/kernel/system/core/include")

# git://github.com/stevenlovegrove/Pangolin.git
find_package(Pangolin 0.1 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Pangolin_LIBRARIES})
message( STATUS "LIB: ${Pangolin_LIBRARIES}" )

find_package(SceneGraph 0.1 REQUIRED)
include_directories(${SceneGraph_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${SceneGraph_LIBRARIES})
message( STATUS "LIB: ${SceneGraph_LIBRARIES}" )

## git clone ssh://robotics.gwu.edu/home/rpg/git/calibu
#find_package( Calibu 0.1 REQUIRED )
#include_directories( ${Calibu_INCLUDE_DIRS} )
#target_link_libraries(${PROJECT_NAME} ${Calibu_LIBRARIES} )
#message( STATUS "LIB: ${Calibu_LIBRARIES}" )

find_package( PbMsgs 0.1 REQUIRED )
include_directories( ${PbMsgs_INCLUDE_DIRS} )
target_link_libraries(${PROJECT_NAME} ${PbMsgs_LIBRARIES} )
message( STATUS "LIB: ${PbMsgs_LIBRARIES}" )

find_package( HAL 0.1 REQUIRED )
include_directories( ${HAL_INCLUDE_DIRS} )
target_link_libraries(${PROJECT_NAME} ${HAL_LIBRARIES} )
message( STATUS "LIB: ${HAL_LIBRARIES}" )

#target_link_libraries( ${PROJECT_NAME}
#    /Users/jmf/Code/Android/Toolchain/user/lib/libceres.a
#    /Users/jmf/Code/Android/Toolchain/user/lib/libminiglog.a
#)
