#ifndef PATH_H
#define PATH_H

#include <iostream>
#include <cstring>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
//#include <sensor_msgs/JointState.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

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

  static geometry_msgs::PoseArray getNodes3D(int width, int height, int depth, int length,
      bool* closed) {
    geometry_msgs::PoseArray nodes;
    nodes.header.frame_id = "path";
    nodes.header.stamp = ros::Time::now();
    int count = 0;
    float max = 0, min = 360;

    for (int i = 0; i < length; ++i) {
      if (closed[i]) {
        count++;
        geometry_msgs::Pose node;
        // center in cell +0.5
        node.position.x = i % width + 0.5;
        node.position.y = (i / width) % height + 0.5;
        // correct for rotation +90/5 not yet done!!!
        node.orientation = tf::createQuaternionMsgFromYaw((i / (width * height) % depth + 18) /
                           (float)180 * M_PI * 5);

        nodes.poses.push_back(node);
      }
    }

    std::cout << count << " 3D nodes expanded" << std::endl;
    return nodes;
  }

  static visualization_msgs::MarkerArray getNodes2D(int width, int height, float* costGoal) {
    visualization_msgs::MarkerArray nodes;

    int count = 0;

    for (int i = 0; i < width * height; ++i) {
      if (costGoal[i] != 0) {
        visualization_msgs::Marker node;

        // delete all previous markers
        if (count == 0) {
          node.action = 3;
        }

        node.header.frame_id = "path";
        node.header.stamp = ros::Time::now();
        node.id = count;
        node.type = visualization_msgs::Marker::SPHERE;
        node.scale.x = 0.1;
        node.scale.y = 0.1;
        node.scale.z = 0.1;
        node.color.a = 1.0;
        node.color.r = 1.0;
        node.color.g = 0.0;
        node.color.b = 0.0;
        // center in cell +0.5
        node.pose.position.x = i % width + 0.5;
        node.pose.position.y = (i / width) % height + 0.5;
        nodes.markers.push_back(node);
        count++;
      }
    }

    std::cout << count << " 2D a* searches conducted" << std::endl;
    return nodes;
  }

 private:
  nav_msgs::Path path;
};

#endif // PATH_H
