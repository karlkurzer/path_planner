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
namespace HybridAStar {
class Path {
 public:
  // ___________
  // CONSTRUCTOR
  Path() {
    // _________________
    // TOPICS TO PUBLISH
    pubPath = n.advertise<nav_msgs::Path>("/path", 1);
    pubPathNodes = n.advertise<visualization_msgs::MarkerArray>("/pathNodes", 1);
    pubPathVehicles = n.advertise<visualization_msgs::MarkerArray>("/pathVehicle", 1);

    // CONFIGURE THE CONTAINER
    path.header.frame_id = "path";
  }

  // __________
  // TRACE PATH
  void tracePath(const Node3D* node, int i = 0);

  // adding a segment to the path
  void addSegment(const Node3D* node);
  // adding a node to the path
  void addNode(const Node3D* node, int i);
  // adding a vehicle shape to the path
  void addVehicle(const Node3D* node, int i);

  // ______________
  // PUBLISH METHODS

  // CLEAR THE PATH
  void clear();
  // PUBLISH THE PATH
  void publishPath() { pubPath.publish(path); }
  // PUBLISH THE NODES OF THE VEHCILE PATH
  void publishPathNodes() { pubPathNodes.publish(pathNodes); }
  // PUBLISH THE VEHICLE ALONG THE PATH
  void publishPathVehicles() { pubPathVehicles.publish(pathVehicles); }

 private:
  ros::NodeHandle n;
  // publisher
  ros::Publisher pubPath;
  ros::Publisher pubPathNodes;
  ros::Publisher pubPathVehicles;
  // path variables
  nav_msgs::Path path;
  visualization_msgs::MarkerArray pathNodes;
  visualization_msgs::MarkerArray pathVehicles;
  // COLORS
  struct color {
    float red;
    float green;
    float blue;
  };
  static constexpr color teal = {102.f / 255.f, 217.f / 255.f, 239.f / 255.f};
  static constexpr color green = {166.f / 255.f, 226.f / 255.f, 46.f / 255.f};
  static constexpr color orange = {253.f / 255.f, 151.f / 255.f, 31.f / 255.f};
  static constexpr color pink = {249.f / 255.f, 38.f / 255.f, 114.f / 255.f};
  static constexpr color purple = {174.f / 255.f, 129.f / 255.f, 255.f / 255.f};
};
}
#endif // PATH_H
