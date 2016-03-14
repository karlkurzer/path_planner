#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include "node3d.h"

class Node3D;

class Visualize {
 public:
  // ___________
  // CONSTRUCTOR
  Visualize() {
    // REGISTER THE PUBLISHER
    pubNode3D = n.advertise<visualization_msgs::Marker>("/node3D", 1000);
    // INITIALIZE THE COUNT
    idNode3D = 0;

  }

  // PUBLISH A SINGE 3D NODE TO RViz
  void publishNode3D(Node3D& node);

 private:
  ros::NodeHandle n;
  ros::Publisher pubNode3D;
  visualization_msgs::MarkerArray nodes3D;
  int idNode3D;
  // COLORS
  struct color {
    float red;
    float green;
    float blue;
  };
  const color teal = {102.f / 255.f, 217.f / 255.f, 239.f / 255.f};

};

#endif // VISUALIZE_H
