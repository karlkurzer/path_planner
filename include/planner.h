#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "constants.h"
#include "helper.h"
#include "collisiondetection.h"
#include "dynamicvoronoi.h"
#include "algorithm.h"
#include "node3d.h"
#include "path.h"
#include "smoother.h"
#include "visualize.h"
#include "lookup.h"

namespace HybridAStar {
class Planner {
 public:
  Planner(rclcpp::Node::SharedPtr n);
  ~Planner() {
    delete[] dubinsLookup;
  }

  void initializeLookups();
  void setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
  void setStart(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr start);
  void setGoal(const geometry_msgs::msg::PoseStamped::SharedPtr goal);
  void plan();

 private:
  rclcpp::Node::SharedPtr n;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubStart;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subMap;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subGoal;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subStart;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> listener;
  geometry_msgs::msg::TransformStamped transform;

  Path path;
  Smoother smoother;
  Path smoothedPath;
  Visualize visualization;
  CollisionDetection configurationSpace;
  DynamicVoronoi voronoiDiagram;

  nav_msgs::msg::OccupancyGrid::SharedPtr grid;
  geometry_msgs::msg::PoseWithCovarianceStamped start;
  geometry_msgs::msg::PoseStamped goal;
  bool validStart = false;
  bool validGoal = false;

  Constants::config collisionLookup[Constants::headings * Constants::positions];
  float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];
};
}
#endif // PLANNER_H
