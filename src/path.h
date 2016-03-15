#ifndef PATH_H
#define PATH_H

#include <iostream>
#include <cstring>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "node3d.h"
#include "constants.h"
#include "helper.h"
#include "gradient.h"

class Path {
 public:
  // ___________
  // CONSTRUCTOR
  Path(Node3D* goal) {
    path.header.frame_id = "path";
    path.header.stamp = ros::Time::now();
    int count = 0;
    tracePath(goal, count);
  }

  // __________
  // TRACE PATH
  void tracePath(const Node3D* node, int count);

  // adding a segment to the path
  void addSegment(const Node3D* node);
  // adding a node to the path
  void addNode(const Node3D* node, int count);
  // adding a vehicle shape to the path
  void addVehicle(const Node3D* node, int count);

  // ______________
  // GETTER METHODS

  // get path
  nav_msgs::Path getPath() {
    return path;
  }
  // get path nodes
  visualization_msgs::MarkerArray getPathNodes() {
    return pathNodes;
  }
  // get path vehicles
  visualization_msgs::MarkerArray getPathVehicles() {
    return pathVehicles;
  }


  // ______________
  // TRACE 3D NODES
  static geometry_msgs::PoseArray getNodes3D(int width, int height, int depth, int length, Node3D* closed);
  // ______________
  // TRACE 2D NODES
  static visualization_msgs::MarkerArray getNodes2D(int width, int height, float* cost2d);
  // ____________
  // COST HEATMAP
  static visualization_msgs::MarkerArray getCosts(int width, int height, int depth, float* cost, float* costToGo);

 private:
  nav_msgs::Path path;
  visualization_msgs::MarkerArray pathNodes;
  visualization_msgs::MarkerArray pathVehicles;
  // COLORS
  struct color {
    float red;
    float green;
    float blue;
  };
  const color teal = {102.f / 255.f, 217.f / 255.f, 239.f / 255.f};
  const color green = {166.f / 255.f, 226.f / 255.f, 46.f / 255.f};
  const color orange = {253.f / 255.f, 151.f / 255.f, 31.f / 255.f};
  const color pink = {249.f / 255.f, 38.f / 255.f, 114.f / 255.f};
  const color purple = {174.f / 255.f, 129.f / 255.f, 255.f / 255.f};
};

#endif // PATH_H
