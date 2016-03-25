#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "gradient.h"

#include "node3d.h"
#include "node2d.h"

class Node3D;
class Node2D;

class Visualize {
 public:
  // ___________
  // CONSTRUCTOR
  Visualize() {
    // _________________
    // TOPICS TO PUBLISH
    pubNode3D = n.advertise<geometry_msgs::PoseStamped>("/visualizeNodes3DPose", 100);
    pubNodes3D = n.advertise<geometry_msgs::PoseArray>("/visualizeNodes3DPoses", 100);
    pubNodes3Dreverse = n.advertise<geometry_msgs::PoseArray>("/visualizeNodes3DPosesReverse", 100);
    pubNodes3DCosts = n.advertise<visualization_msgs::MarkerArray>("/visualizeNodes3DCosts", 100);
    pubNode2D = n.advertise<geometry_msgs::PoseStamped>("/visualizeNodes2DPose", 100);
    pubNodes2D = n.advertise<geometry_msgs::PoseArray>("/visualizeNodes2DPoses", 100);
    pubNodes2DCosts = n.advertise<visualization_msgs::MarkerArray>("/visualizeNodes2DCosts", 100);

    // CONFIGURE THE CONTAINER
    poses3D.header.frame_id = "path";
    poses3Dreverse.header.frame_id = "path";
    poses2D.header.frame_id = "path";
  }

  // CLEAR VISUALIZATION
  void clear();
  void clear2D() {poses2D.poses.clear();}

  // PUBLISH A SINGEL/ARRAY 3D NODE TO RViz
  void publishNode3DPose(Node3D& node);
  void publishNode3DPoses(Node3D& node);
  // PUBLISH THE COST FOR A 3D NODE TO RViz
  void publishNode3DCosts(Node3D* nodes, int width, int height, int depth);

  // PUBLISH A SINGEL/ARRAY 2D NODE TO RViz
  void publishNode2DPose(Node2D& node);
  void publishNode2DPoses(Node2D& node);
  // PUBLISH THE COST FOR A 2D NODE TO RViz
  void publishNode2DCosts(Node2D* nodes, int width, int height);

 private:
  ros::NodeHandle n;
  //publisher
  ros::Publisher pubNode3D;
  ros::Publisher pubNodes3D;
  ros::Publisher pubNodes3Dreverse;
  ros::Publisher pubNode2D;
  ros::Publisher pubNodes2D;
  ros::Publisher pubNodes3DCosts;
  ros::Publisher pubNodes2DCosts;
  // visualization variables
  geometry_msgs::PoseArray poses3D;
  geometry_msgs::PoseArray poses3Dreverse;
  geometry_msgs::PoseArray poses2D;
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
