#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "node3d.h"

class Node3D;

class Visualize {
 public:
  // ___________
  // CONSTRUCTOR
  Visualize() {
    // REGISTER THE PUBLISHER
    //    pubNode3D = n.advertise<visualization_msgs::MarkerArray>("/visualizeNodes3D", 1);
    pubNode3D = n.advertise<geometry_msgs::PoseStamped>("/visualizeNodes3DPose", 100);
    pubNodes3D = n.advertise<geometry_msgs::PoseArray>("/visualizeNodes3DPoses", 100);
    // INITIALIZE THE COUNT
    idNode3D = 0;

    // CONFIGURE THE CONTAINER
    poses3D.header.frame_id = "path";
    poses3D.header.stamp = ros::Time();
  }

  // PUBLISH A SINGEL/ARRAY 3D NODE TO RViz
  void publishNode3DPose(Node3D& node);
  void publishNode3DPoses(Node3D& node);

 private:
  ros::NodeHandle n;
  ros::Publisher pubNode3D;
  ros::Publisher pubNodes3D;
  visualization_msgs::MarkerArray nodes3D;
  geometry_msgs::PoseArray poses3D;
  int idNode3D;
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

#endif // VISUALIZE_H
