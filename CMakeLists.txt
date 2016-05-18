cmake_minimum_required(VERSION 2.8.3)
project(hybrid_astar)


## C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Werror")
elseif(COMPILER_SUPPORTS_CXX0X)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

file(GLOB SOURCES
    src/dubins.cpp
    include/dubins.h
    include/gradient.h
    src/node2d.cpp
    include/node2d.h
    src/node3d.cpp
    include/node3d.h
    src/algorithm.cpp
    include/algorithm.h
    src/path.cpp
    include/path.h
    src/visualize.cpp
    include/visualize.h
    src/collisiondetection.cpp
    include/collisiondetection.h
    include/lookup.h
    include/constants.h
    include/helper.h
    include/subscribeandpublish.h
)

#just for QT-creator
set(PUBLIC_INCLUDE
#include/HybridAStar/constants.h
#include/HybridAStar/gradient.h
#include/HybridAStar/hybrid_astar.h
#include/HybridAStar/node2d.h
#include/HybridAStar/path.h
#include/HybridAStar/dubins.h
#include/HybridAStar/helper.h
#include/HybridAStar/lookup.h
#include/HybridAStar/node3d.h
#include/HybridAStar/subscribeandpublish.h
#include/HybridAStar/visualize.h
)

## Declare a catkin package
catkin_package()

### OPEN MOTION PLANNING LIBRARY
#find_package(OMPL REQUIRED)

#if(NOT OMPL_FOUND)
#    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
#endif(NOT OMPL_FOUND)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${OMPL_INCLUDE_DIRS})

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

add_executable(hybrid_astar src/main.cpp ${SOURCES})
target_link_libraries(hybrid_astar ${catkin_LIBRARIES})
target_link_libraries(hybrid_astar ${OMPL_LIBRARIES})
