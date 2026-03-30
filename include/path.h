#ifndef PATH_H
#define PATH_H

#include <iostream>
#include <cstring>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "node3d.h"
#include "constants.h"
#include "helper.h"
namespace HybridAStar {
class Path {
 public:
  Path() {}
  Path(rclcpp::Node::SharedPtr n, bool smoothed = false) : n(n) {
    std::string pathTopic = "/path";
    std::string pathNodesTopic = "/pathNodes";
    std::string pathVehicleTopic = "/pathVehicle";

    if (smoothed) {
      pathTopic = "/sPath";
      pathNodesTopic = "/sPathNodes";
      pathVehicleTopic = "/sPathVehicle";
      this->smoothed = smoothed;
    }

    pubPath = n->create_publisher<nav_msgs::msg::Path>(pathTopic, 1);
    pubPathNodes = n->create_publisher<visualization_msgs::msg::MarkerArray>(pathNodesTopic, 1);
    pubPathVehicles = n->create_publisher<visualization_msgs::msg::MarkerArray>(pathVehicleTopic, 1);

    path.header.frame_id = "path";
  }

  void updatePath(const std::vector<Node3D> &nodePath);
  void addSegment(const Node3D& node);
  void addNode(const Node3D& node, int i);
  void addVehicle(const Node3D& node, int i);

  void clear();
  void publishPath() { pubPath->publish(path); }
  void publishPathNodes() { pubPathNodes->publish(pathNodes); }
  void publishPathVehicles() { pubPathVehicles->publish(pathVehicles); }

 private:
  rclcpp::Node::SharedPtr n;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubPathNodes;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubPathVehicles;
  nav_msgs::msg::Path path;
  visualization_msgs::msg::MarkerArray pathNodes;
  visualization_msgs::msg::MarkerArray pathVehicles;
  bool smoothed = false;
};
}
#endif // PATH_H
