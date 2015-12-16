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
    sub_goal = n.subscribe("/move_base_simple/goal", 1, &SubscribeAndPublish::setGoal, this);
  }

  void setMap(const nav_msgs::OccupancyGrid::ConstPtr map) {
    std::cout << "I heard something:" << std::endl;
    grid = map;
  }

  void setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal) {
    std::cout << "I am seeing a new goal:" << std::endl;
    int x = (int)goal->pose.position.x;
    int y = (int)goal->pose.position.y;
    Node3D nStart(0, 0, 180, 0, 0, nullptr);
    Node3D nGoal(x, y, 180, 0, 0, nullptr);
    Path path(Node3D::aStar(nStart, nGoal, grid), "path");
    pub_path.publish(path.getPath());
  }

 private:
  ros::NodeHandle n;
  ros::Publisher pub_path;
  ros::Subscriber sub_map;
  ros::Subscriber sub_goal;
  nav_msgs::OccupancyGrid::ConstPtr grid;
};

#endif // SUBSCRIBEANDPUBLISH



