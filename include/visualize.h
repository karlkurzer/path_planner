#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "gradient.h"
#include "node3d.h"
#include "node2d.h"

namespace HybridAStar {
class Node3D;
class Node2D;

class Visualize {
 public:
  Visualize() {}
  Visualize(rclcpp::Node::SharedPtr n) : n(n) {
    pubNode3D = n->create_publisher<geometry_msgs::msg::PoseStamped>("/visualizeNodes3DPose", 100);
    pubNodes3D = n->create_publisher<geometry_msgs::msg::PoseArray>("/visualizeNodes3DPoses", 100);
    pubNodes3Dreverse = n->create_publisher<geometry_msgs::msg::PoseArray>("/visualizeNodes3DPosesReverse", 100);
    pubNodes3DCosts = n->create_publisher<visualization_msgs::msg::MarkerArray>("/visualizeNodes3DCosts", 100);
    pubNode2D = n->create_publisher<geometry_msgs::msg::PoseStamped>("/visualizeNodes2DPose", 100);
    pubNodes2D = n->create_publisher<geometry_msgs::msg::PoseArray>("/visualizeNodes2DPoses", 100);
    pubNodes2DCosts = n->create_publisher<visualization_msgs::msg::MarkerArray>("/visualizeNodes2DCosts", 100);

    poses3D.header.frame_id = "path";
    poses3Dreverse.header.frame_id = "path";
    poses2D.header.frame_id = "path";
  }

  void clear();
  void clear2D() {poses2D.poses.clear();}

  void publishNode3DPose(Node3D& node);
  void publishNode3DPoses(Node3D& node);
  void publishNode3DCosts(Node3D* nodes, int width, int height, int depth);

  void publishNode2DPose(Node2D& node);
  void publishNode2DPoses(Node2D& node);
  void publishNode2DCosts(Node2D* nodes, int width, int height);

 private:
  rclcpp::Node::SharedPtr n;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubNode3D;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pubNodes3D;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pubNodes3Dreverse;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubNodes3DCosts;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubNode2D;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pubNodes2D;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubNodes2DCosts;
  geometry_msgs::msg::PoseArray poses3D;
  geometry_msgs::msg::PoseArray poses3Dreverse;
  geometry_msgs::msg::PoseArray poses2D;
};
}
#endif // VISUALIZE_H
