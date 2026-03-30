cmake_minimum_required(VERSION 3.8)
project(hybrid_astar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(ompl REQUIRED)
if(NOT ompl_FOUND)
    message(AUTHOR_WARNING "Open Motion Planning Library not found")
endif()

set(SOURCES
    src/algorithm.cpp
    src/node2d.cpp
    src/node3d.cpp
    src/collisiondetection.cpp
    src/planner.cpp
    src/path.cpp
    src/smoother.cpp
    src/visualize.cpp
    src/dubins.cpp
    src/dynamicvoronoi.cpp
    src/bucketedqueue.cpp
)

set(HEADERS
    include/algorithm.h
    include/node2d.h
    include/node3d.h
    include/collisiondetection.h
    include/planner.h
    include/path.h
    include/smoother.h
    include/vector2d.h
    include/visualize.h
    include/helper.h
    include/constants.h
    include/lookup.h
    include/gradient.h
    include/dubins.h
    include/dynamicvoronoi.h
    include/bucketedqueue.h
    include/point.h
)

include_directories(
    /usr/include/ompl-1.5
    include
)

add_library(${PROJECT_NAME}_lib ${SOURCES})
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

if(ompl_FOUND)
  target_link_libraries(${PROJECT_NAME}_lib ompl)
endif()

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster rclcpp nav_msgs tf2 tf2_ros geometry_msgs)

add_executable(hybrid_astar src/main.cpp)
target_link_libraries(hybrid_astar ${PROJECT_NAME}_lib ompl)
ament_target_dependencies(hybrid_astar rclcpp)

install(TARGETS ${PROJECT_NAME}_lib tf_broadcaster hybrid_astar
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/ maps/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_parity test/test_parity.cpp)
  target_link_libraries(test_parity ${PROJECT_NAME}_lib ompl)
  ament_target_dependencies(test_parity rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs ament_cmake_gtest)

  # For AddressSanitizer and UndefinedBehaviorSanitizer
  target_compile_options(test_parity PRIVATE -O0 -g -fno-omit-frame-pointer -fsanitize=address,undefined)
  target_link_options(test_parity PRIVATE -fsanitize=address,undefined)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs geometry_msgs nav_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs)

ament_package()
