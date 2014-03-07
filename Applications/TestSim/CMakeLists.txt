project(TestSim)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

find_package(Pangolin 0.1 QUIET)
find_package(HAL 0.1 QUIET)
#find_package(Calibu)
find_package(Node)

if(Pangolin_FOUND AND HAL_FOUND AND Node_FOUND)
  option(BUILD_TestSim "To build TestSim or not build TestSim marks the variable" OFF)
  mark_as_advanced(BUILD_TestSim)
  add_executable( ${PROJECT_NAME} main.cpp )

  include_directories( ${HAL_INCLUDE_DIRS} )
  include_directories( ${Pangolin_INCLUDE_DIRS} )
  target_link_libraries(${PROJECT_NAME}
    ${Pangolin_LIBRARIES}
    #${Calibu_LIBRARIES}
    ${HAL_LIBRARIES})
endif()
