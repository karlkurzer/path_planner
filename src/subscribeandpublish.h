#ifndef SUBSCRIBEANDPUBLISH
#define SUBSCRIBEANDPUBLISH

#include <iostream>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "node3d.h"
#include "path.h"

class SubscribeAndPublish {
 public:
  SubscribeAndPublish() {
    //Topic you want to publish
    pub_path = n.advertise<nav_msgs::Path>("/path", 1);
    //Topic you want to subscribe
    sub_map = n.subscribe("/map", 1, &SubscribeAndPublish::setMap, this);
    sub_goal = n.subscribe("/move_base_simple/goal", 1,
                           &SubscribeAndPublish::setGoal, this);
    sub_start = n.subscribe("/initialpose", 1,
                            &SubscribeAndPublish::setStart, this);
  }

  void setMap(const nav_msgs::OccupancyGrid::ConstPtr map) {
    std::cout << "I am seeing the map..." << std::endl;
    grid = map;
  }

  void setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal) {
    int x = (int)goal->pose.position.x;
    int y = (int)goal->pose.position.y;
    float t = tf::getYaw(goal->pose.orientation);

    if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
      Node3D nGoal(x, y, t * 180 / 3.14, 0, 0, nullptr);
      Node3D nStart(0, 0, 0, 0, 0, nullptr);

      if (start != nullptr) {
        nStart.setX(start->pose.pose.position.x);
        nStart.setY(start->pose.pose.position.y);
        nStart.setT(tf::getYaw(start->pose.pose.orientation) * 180 / 3.14);
      }

      std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:"
                << t << " deg:" << t * 180 / 3.14 << std::endl;
      Path path(Node3D::aStar(nStart, nGoal, grid), "path");
      pub_path.publish(path.getPath());
    } else {
      std::cout << "Invalid goal selected x:" << x << " y:" << y << " t:"
                << t << " deg:" << t * 180 / 3.14 << std::endl;
    }
  }

  void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
    int x = (int)initial->pose.pose.position.x;
    int y = (int)initial->pose.pose.position.y;
    float t = tf::getYaw(initial->pose.pose.orientation);
    std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:"
              << t << " deg:" << t * 180 / 3.14 << std::endl;

    if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
      start = initial;
    } else {
      std::cout << "Invalid start selected x:" << x << " y:" << y << " t:"
                << t << " deg:" << t * 180 / 3.14 << std::endl;
    }
  }

 private:
  ros::NodeHandle n;
  ros::Publisher pub_path;
  ros::Subscriber sub_map;
  ros::Subscriber sub_goal;
  ros::Subscriber sub_start;
  nav_msgs::OccupancyGrid::ConstPtr grid;
  geometry_msgs::PoseWithCovarianceStamped::ConstPtr start;
};

#endif // SUBSCRIBEANDPUBLISH



