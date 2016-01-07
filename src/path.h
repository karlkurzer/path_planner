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

#include "node3d.h"


class Path {
 public:
  // CONSTRUCTOR
  Path(Node3D* goal, const std::string& frame_id) {
    path.header.frame_id = frame_id;
    path.header.stamp = ros::Time::now();
    tracePath(goal);
  }

  nav_msgs::Path getPath() {
    return path;
  }

  void tracePath(Node3D* node) {
    if (node == nullptr) { return; }

    addNode(node);
    tracePath(node->getPred());
  }

  void addNode(Node3D* node) {
    geometry_msgs::PoseStamped vertex;
    vertex.pose.position.x = node->getX();
    vertex.pose.position.y = node->getY();
    vertex.pose.position.z = 0;
    vertex.pose.orientation.x = 0;
    vertex.pose.orientation.y = 0;
    vertex.pose.orientation.z = 0;
    vertex.pose.orientation.w = 0;
    path.poses.push_back(vertex);
  }

  static geometry_msgs::PoseArray getNodes(int width, int height, int depth, int length, bool* closed) {
    geometry_msgs::PoseArray nodes;
    nodes.header.frame_id = "path";
    nodes.header.stamp = ros::Time::now();
    int count = 0;

    for (int i = 0; i < length; ++i) {
      if (closed[i]) {
        count++;
        geometry_msgs::Pose node;
        node.position.x = i % width;
        node.position.y = (i / width) % height;
        node.orientation = tf::createQuaternionMsgFromYaw((float)Node3D::dt[(i /
                           (width * height)) % depth] / 180 * M_PI);
        nodes.poses.push_back(node);
      }
    }

    std::cout << count << " nodes expanded" << std::endl;
    return nodes;
  }

 private:
  nav_msgs::Path path;
};

#endif // PATH_H
