#ifndef SUBSCRIBEANDPUBLISH
#define SUBSCRIBEANDPUBLISH

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "constants.h"
#include "helper.h"
#include "node3d.h"
#include "path.h"
#include "lookup.h"

class SubscribeAndPublish {
 public:
  //###################################################
  //                                        CONSTRUCTOR
  //###################################################
  SubscribeAndPublish() {
    // _____
    // TODOS
    if (constants::dubinsLookup) {
      lookup::dubinsLookup(dubinsLookup);
    }

    lookup::collisionLookup(collisionLookup);

    // _________________
    // TOPICS TO PUBLISH
    //    pub_rcv = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    pub_path = n.advertise<nav_msgs::Path>("/path", 1);
    pub_pathNodes = n.advertise<visualization_msgs::MarkerArray>("/pathNodes", 1);
    pub_pathVehicles = n.advertise<visualization_msgs::MarkerArray>("/pathVehicles", 1);
    pub_nodes3D = n.advertise<geometry_msgs::PoseArray>("/nodes3D", 1);
    pub_nodes2D = n.advertise<visualization_msgs::MarkerArray>("/nodes2D", 1);
    pub_costCubes = n.advertise<visualization_msgs::MarkerArray>("/costCubes", 1);
    pub_start = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);

    // ___________________
    // TOPICS TO SUBSCRIBE
    if (constants::manual) {
      sub_map = n.subscribe("/map", 1, &SubscribeAndPublish::setMap, this);
    } else {
      sub_map = n.subscribe("/occ_map", 1, &SubscribeAndPublish::setMap, this);
    }

    sub_goal = n.subscribe("/move_base_simple/goal", 1, &SubscribeAndPublish::setGoal, this);
    sub_start = n.subscribe("/initialpose", 1, &SubscribeAndPublish::setStart, this);
  }

  //###################################################
  //                                                MAP
  //###################################################
  void setMap(const nav_msgs::OccupancyGrid::Ptr map) {
    if (constants::coutDEBUG) {
      std::cout << "I am seeing the map..." << std::endl;
    }

    grid = map;

    if (constants::obstacleBloating) {
      bloatObstacles(grid);
    }

    // plan if the switch is not set to manual
    if (!constants::manual) { plan();}
  }

  //###################################################
  //                                      PLAN THE PATH
  //###################################################
  void plan() {
    // if a start as well as goal are defined go ahead and plan
    if (start != nullptr && goal != nullptr) {
      // ___________________________
      // LISTS ALLOWCATED ROW MAJOR ORDER
      int width = grid->info.width;
      int height = grid->info.height;
      int depth = constants::headings;
      int length = width * height * depth;
      // define list pointers and initialize lists
      Node3D* nodes = new Node3D[length]();
      float* cost2d = new float[width * height]();


      // ________________________
      // retrieving goal position
      float x = goal->pose.position.x;
      float y = goal->pose.position.y;
      // adjust the theta for the RViz offset
      float t = tf::getYaw(goal->pose.orientation);

      // set theta to a value (0,2PI]
      t = helper::normalizeHeadingRad(t);

      Node3D nGoal(x, y, t, 0, 0, nullptr);

      // _________________________
      // retrieving start position
      x = start->pose.pose.position.x;
      y = start->pose.pose.position.y;
      // adjust the theta for the RViz offset
      t = tf::getYaw(start->pose.pose.orientation);

      // set theta to a value (0,2PI]
      t = helper::normalizeHeadingRad(t);

      Node3D nStart(x, y, t, 0, 0, nullptr);

      // ___________________________
      // START AND TIME THE PLANNING
      ros::Time t0 = ros::Time::now();
      Path path(Node3D::aStar(nStart, nGoal, nodes, cost2d, grid, collisionLookup, dubinsLookup), "path");
      ros::Time t1 = ros::Time::now();
      ros::Duration d(t1 - t0);

      if (constants::coutDEBUG) {
        std::cout << "Time in ms: " << d * 1000 << std::endl;
      }

      // _________________________________
      // PUBLISH THE RESULTS OF THE SEARCH
      pub_path.publish(path.getPath());
      pub_pathNodes.publish(path.getPathNodes());
      pub_pathVehicles.publish(path.getPathVehicles());
//      pub_nodes3D.publish(Path::getNodes3D(width, height, depth, length, nodes));
//      pub_nodes2D.publish(Path::getNodes2D(width, height, cost2d));
//      pub_costCubes.publish(Path::getCosts(width, height, depth, cost, costToGo));

      delete [] nodes;
      nodes = nullptr;
      delete [] cost2d;

    } else {
      std::cout << "missing goal or start" << std::endl;
    }
  }

  //###################################################
  //                                    INITIALIZE GOAL
  //###################################################
  void setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {
    // retrieving goal position
    float x = end->pose.position.x;
    float y = end->pose.position.y;
    float t = tf::getYaw(end->pose.orientation);

    std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << helper::toDeg(t) << std::endl;

    if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
      goal = end;

      if (constants::manual) { plan();}

    } else {
      std::cout << "invalid goal x:" << x << " y:" << y << " t:" << helper::toDeg(t) << std::endl;
    }
  }

  //###################################################
  //                                   INITIALIZE START
  //###################################################
  void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
    float x = initial->pose.pose.position.x;
    float y = initial->pose.pose.position.y;
    float t = tf::getYaw(initial->pose.pose.orientation);
    // publish the start without covariance for rviz
    geometry_msgs::PoseStamped startN;
    startN.pose.position = initial->pose.pose.position;
    startN.pose.orientation = initial->pose.pose.orientation;
    startN.header.frame_id = "map";
    startN.header.stamp = ros::Time::now();

    std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << helper::toDeg(t) << std::endl;

    if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
      start = initial;

      if (constants::manual) { plan();}

      // publish start for RViz
      pub_start.publish(startN);
    } else {
      std::cout << "invalid start x:" << x << " y:" << y << " t:" << helper::toDeg(t) << std::endl;
    }
  }

  //###################################################
  //                                   OBSTACLEBLOATING
  //###################################################
  void bloatObstacles(nav_msgs::OccupancyGrid::Ptr& grid) {
    int dx[] = { 1,  1,  0,  -1,  -1, -1,   0,    1 };
    int dy[] = { 0,  1,  1,   1,   0, -1,  -1,   -1 };
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
        for (int j = 0; j < 8; ++j) {
          xSucc = x + dx[j];
          ySucc = y + dy[j];

          if (xSucc >= 0 && xSucc < width && ySucc >= 0 && ySucc < height) {
            bloating[ySucc * width + xSucc] = true;
          }
        }
      }
    }

    for (int i = 0; i < length; ++i) {
      grid->data[i] = bloating[i];
    }

    delete [] bloating;
  }

 private:
  ros::NodeHandle n;
  // publisher
  //  ros::Publisher pub_rcv;
  ros::Publisher pub_path;
  ros::Publisher pub_pathNodes;
  ros::Publisher pub_pathVehicles;
  ros::Publisher pub_nodes3D;
  ros::Publisher pub_nodes2D;
  ros::Publisher pub_costCubes;
  ros::Publisher pub_start;
  // subscriber
  ros::Subscriber sub_map;
  ros::Subscriber sub_goal;
  ros::Subscriber sub_start;
  // general pointer
  nav_msgs::OccupancyGrid::Ptr grid;
  geometry_msgs::PoseWithCovarianceStamped::ConstPtr start;
  geometry_msgs::PoseStamped::ConstPtr goal;
  constants::config collisionLookup[constants::headings * constants::positions];
  float* dubinsLookup = new float [constants::headings * constants::headings * constants::dubinsWidth * constants::dubinsWidth];
};

#endif // SUBSCRIBEANDPUBLISH



