#ifndef SUBSCRIBEANDPUBLISH
#define SUBSCRIBEANDPUBLISH

#include <iostream>

#include <ros/ros.h>

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
  }

  void setMap(const nav_msgs::OccupancyGrid::ConstPtr map) {
    std::cout << "I am seeing the map..." << std::endl;
    grid = map;
  }

  void setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal) {
    int x = (int)goal->pose.position.x;
    int y = (int)goal->pose.position.y;
    float t = goal->pose.orientation.z;
    float w = goal->pose.orientation.w;
    std::cout << "I am seeing a new goal x:" << x << " y:" << y
              << " t:" << 2 * asin(t) << " w:" << 2 * acos(w) << std::endl;
    Node3D nStart(0, 0, 180, 0, 0, nullptr);

    if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
      Node3D nGoal(x, y, 180, 0, 0, nullptr);
      Path path(Node3D::aStar(nStart, nGoal, grid), "path");
      pub_path.publish(path.getPath());
    } else {
      std::cout << "Invalid goal selected x:" << x << " y:" << y
                << " t:" << 2 * asin(t) << " w:" << 2 * acos(w) << std::endl;
    }
  }

 private:
  ros::NodeHandle n;
  ros::Publisher pub_path;
  ros::Subscriber sub_map;
  ros::Subscriber sub_goal;
  nav_msgs::OccupancyGrid::ConstPtr grid;
};

#endif // SUBSCRIBEANDPUBLISH



