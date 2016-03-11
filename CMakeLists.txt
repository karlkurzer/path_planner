cmake_minimum_required(VERSION 2.8.3)
project(path_planner)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
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
    src/path.cpp
    src/path.h
    src/lookup.h
    src/constants.h
    src/helper.h
    src/subscribeandpublish.h
)

## Declare ROS messages and services

## Generate added messages and services

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

add_executable(a_star src/main.cpp ${SOURCES})
target_link_libraries(a_star ${catkin_LIBRARIES})
