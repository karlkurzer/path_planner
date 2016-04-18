cmake_minimum_required(VERSION 2.8.3)
project(path_planner)


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
    src/dubins.h
    src/gradient.h
    src/node2d.cpp
    src/node2d.h
    src/node3d.cpp
    src/node3d.h
    src/algorithm.cpp
    src/algorithm.h
    src/path.cpp
    src/path.h
    src/visualize.cpp
    src/visualize.h
    src/lookup.h
    src/constants.h
    src/helper.h
    src/subscribeandpublish.h
)

## Declare a catkin package
catkin_package()

## OPEN MOTION PLANNING LIBRARY
find_package(OMPL REQUIRED)

if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${OMPL_INCLUDE_DIRS})

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

add_executable(a_star src/main.cpp ${SOURCES})
target_link_libraries(a_star ${catkin_LIBRARIES})
target_link_libraries(a_star ${OMPL_LIBRARIES})
