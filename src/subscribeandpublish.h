#ifndef SUBSCRIBEANDPUBLISH
#define SUBSCRIBEANDPUBLISH

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "node3d.h"
#include "path.h"

class SubscribeAndPublish {
 public:
  //###################################################
  //                                        CONSTRUCTOR
  //###################################################
  SubscribeAndPublish() {
    // topics to publish
    pub_path = n.advertise<nav_msgs::Path>("/path", 1);
    pub_nodes3D = n.advertise<geometry_msgs::PoseArray>("/nodes3D", 1);
    pub_nodes2D = n.advertise<visualization_msgs::MarkerArray>("/nodes2D", 1);
    // topics to subscribe
    sub_map = n.subscribe("/map", 1, &SubscribeAndPublish::setMap, this);
    sub_goal = n.subscribe("/move_base_simple/goal", 1, &SubscribeAndPublish::setGoal, this);
    sub_start = n.subscribe("/initialpose", 1, &SubscribeAndPublish::setStart, this);
  }

  //###################################################
  //                                                MAP
  //###################################################
  void setMap(const nav_msgs::OccupancyGrid::Ptr map) {
    std::cout << "I am seeing the map..." << std::endl;
    grid = map;
    bloatObstacles(grid);
  }

  //###################################################
  //                         INITIALIZE GOAL AND SEARCH
  //###################################################
  void setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal) {

    // LISTS dynamically allocated ROW MAJOR ORDER
    int width = grid->info.width;
    int height = grid->info.height;
    int depth = 8;
    int length = width * height * depth;
    bool* open;
    bool* closed;
    float* cost;
    float* costToGo;
    float* costGoal;
    // initialize all lists
    open = new bool [length]();
    closed = new bool [length]();
    cost = new float [length]();
    costToGo = new float [length]();
    // 2D COSTS
    costGoal = new float [width * height]();

    // retrieving goal position
    int x = (int)goal->pose.position.x;
    int y = (int)goal->pose.position.y;
    float t = tf::getYaw(goal->pose.orientation) * 180 / M_PI;

    if (t < 0) {
      t = 360 + t;
    }

    if (height >= y && y >= 0 && width >= x && x >= 0) {
      Node3D nGoal(x, y, t, 0, 0, nullptr);
      std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << t << std::endl;
      Node3D nStart(0, 0, 0, 0, 0, nullptr);

      // setting the start position and catches an undefined error if neccessary
      if (start != nullptr) {
        nStart.setX(start->pose.pose.position.x);
        nStart.setY(start->pose.pose.position.y);
        t = tf::getYaw(start->pose.pose.orientation) * 180 / M_PI;

        if (t < 0) {
          t = 360 + t;
        }

        nStart.setT(t);
      }

      //      clock_t t1, t2;
      //      t1 = clock();
      Path path(Node3D::aStar(nStart, nGoal, grid, width, height, depth, length, open, closed, cost,
                              costToGo, costGoal), "path");
      //      t2 = clock();
      //      std::cout << "time: " << x << ((float)t2 - (float)t1) / 1000000 << std::endl;

      // publish the results of the search
      pub_path.publish(path.getPath());
      pub_nodes3D.publish(Path::getNodes3D(width, height, depth, length, closed));
      pub_nodes2D.publish(Path::getNodes2D(width, height, costGoal));

      // LISTS deleted
      delete[] open;
      delete[] closed;
      delete[] cost;
      delete[] costToGo;
      delete[] costGoal;
    } else {
      std::cout << "invalid goal x:" << x << " y:" << y << " t:" << t << std::endl;
    }
  }

  //###################################################
  //                                   INITIALIZE START
  //###################################################
  void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
    int x = (int)initial->pose.pose.position.x;
    int y = (int)initial->pose.pose.position.y;
    float t = tf::getYaw(initial->pose.pose.orientation) * 180 / M_PI;

    if (t < 0) {
      t = 360 + t;
    }

    std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << t << std::endl;

    if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
      start = initial;
    } else {
      std::cout << "invalid start x:" << x << " y:" << y << " t:" << t << std::endl;
    }
  }

  //###################################################
  //                                   OBSTACLEBLOATING
  //###################################################
  void bloatObstacles(nav_msgs::OccupancyGrid::Ptr& grid) {
    int height = grid->info.height;
    int width = grid->info.width;
    int length = height * width;
    int x, y, xSucc, ySucc;
    bool* bloating;
    bloating = new bool[length]();

    for (int i = 0; i < length; ++i) {
      x = i % width;
      y = (i / width) % height;

      if (grid->data[i]) {
        // bloat the obstacle
        for (int i = 0; i < 8; ++i) {
          xSucc = x + Node3D::dx[i];
          ySucc = y + Node3D::dy[i];

          if (xSucc >= 0 && xSucc < width && ySucc >= 0 && ySucc < height) {
            bloating[ySucc * width + xSucc] = true;
          }
        }
      }
    }

    for (int i = 0; i < length; ++i) {
      grid->data[i] = bloating[i];
    }

    delete[] bloating;
  }

 private:
  ros::NodeHandle n;
  ros::Publisher pub_path;
  ros::Publisher pub_nodes3D;
  ros::Publisher pub_nodes2D;
  ros::Subscriber sub_map;
  ros::Subscriber sub_goal;
  ros::Subscriber sub_start;
  nav_msgs::OccupancyGrid::Ptr grid;
  geometry_msgs::PoseWithCovarianceStamped::ConstPtr start;
};

#endif // SUBSCRIBEANDPUBLISH



