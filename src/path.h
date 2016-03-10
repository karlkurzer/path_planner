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
#include <sensor_msgs/PointCloud2.h>

#include "node3d.h"
#include "constants.h"

class Path {
 public:
  // ___________
  // CONSTRUCTOR
  Path(Node3D* goal, const std::string& frame_id) {
    path.header.frame_id = frame_id;
    path.header.stamp = ros::Time::now();
    int count = 0;
    tracePath(goal, count);
  }

  // __________
  // TRACE PATH
  void tracePath(Node3D* node, int count);

  // addign a segment to the path
  void addSegment(Node3D* node);
  // adding a node to the path
  void addNode(Node3D* node, int count);
  // adding a vehicle shape to the path
  void addVehicle(Node3D* node, int count);

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
  static geometry_msgs::PoseArray getNodes3D(int width, int height, int depth, int length, bool* closed);
  // ______________
  // TRACE 2D NODES
  static visualization_msgs::MarkerArray getNodes2D(int width, int height, float* cost2d);
  // ____________
  // COST HEATMAP
  static sensor_msgs::PointCloud2 getCosts(int width, int height, int depth, float* cost, float* costToGo);

 private:
  nav_msgs::Path path;
  visualization_msgs::MarkerArray pathNodes;
  visualization_msgs::MarkerArray pathVehicles;
};

#endif // PATH_H
