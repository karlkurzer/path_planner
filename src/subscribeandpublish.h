#ifndef SUBSCRIBEANDPUBLISH
#define SUBSCRIBEANDPUBLISH

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "constants.h"
#include "helper.h"
#include "algorithm.h"
#include "node3d.h"
#include "path.h"
#include "visualize.h"
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
    //    pub_nodes2D = n.advertise<visualization_msgs::MarkerArray>("/nodes2D", 1);
    pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);

    // ___________________
    // TOPICS TO SUBSCRIBE
    if (constants::manual) {
      subMap = n.subscribe("/map", 1, &SubscribeAndPublish::setMap, this);
    } else {
      subMap = n.subscribe("/occ_map", 1, &SubscribeAndPublish::setMap, this);
    }

    subGoal = n.subscribe("/move_base_simple/goal", 1, &SubscribeAndPublish::setGoal, this);
    subStart = n.subscribe("/initialpose", 1, &SubscribeAndPublish::setStart, this);
  }

  //###################################################
  //                                                MAP
  //###################################################
  void setMap(const nav_msgs::OccupancyGrid::Ptr map) {
    if (constants::coutDEBUG) {
      std::cout << "I am seeing the map..." << std::endl;
    }

    grid = map;

    // plan if the switch is not set to manual and a transform is available
    if (!constants::manual && listener.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr)) {

      listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

      // assign the values to start from base_link
      start.pose.pose.position.x = transform.getOrigin().x();
      start.pose.pose.position.y = transform.getOrigin().y();
      tf::quaternionTFToMsg(transform.getRotation(), start.pose.pose.orientation);

      if (grid->info.height >= start.pose.pose.position.y && start.pose.pose.position.y >= 0 &&
          grid->info.width >= start.pose.pose.position.x && start.pose.pose.position.x >= 0) {
        // set the start as valid and plan
        validStart = true;
      } else  {
        validStart = false;
      }

      plan();
    }
  }

  //###################################################
  //                                   INITIALIZE START
  //###################################################
  void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
    float x = initial->pose.pose.position.x / constants::cellSize;
    float y = initial->pose.pose.position.y / constants::cellSize;
    float t = tf::getYaw(initial->pose.pose.orientation);
    // publish the start without covariance for rviz
    geometry_msgs::PoseStamped startN;
    startN.pose.position = initial->pose.pose.position;
    startN.pose.orientation = initial->pose.pose.orientation;
    startN.header.frame_id = "map";
    startN.header.stamp = ros::Time::now();

    std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << helper::toDeg(t) << std::endl;

    if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
      validStart = true;
      start = *initial;

      if (constants::manual) { plan();}

      // publish start for RViz
      pubStart.publish(startN);
    } else {
      std::cout << "invalid start x:" << x << " y:" << y << " t:" << helper::toDeg(t) << std::endl;
    }
  }

  //###################################################
  //                                    INITIALIZE GOAL
  //###################################################
  void setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {
    // retrieving goal position
    float x = end->pose.position.x / constants::cellSize;
    float y = end->pose.position.y / constants::cellSize;
    float t = tf::getYaw(end->pose.orientation);

    std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << helper::toDeg(t) << std::endl;

    if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
      validGoal = true;
      goal = *end;

      if (constants::manual) { plan();}

    } else {
      std::cout << "invalid goal x:" << x << " y:" << y << " t:" << helper::toDeg(t) << std::endl;
    }
  }


  //###################################################
  //                                      PLAN THE PATH
  //###################################################
  void plan() {
    // if a start as well as goal are defined go ahead and plan
    if (validStart && validGoal) {

      // ___________________________
      // LISTS ALLOWCATED ROW MAJOR ORDER
      int width = grid->info.width;
      int height = grid->info.height;
      int depth = constants::headings;
      int length = width * height * depth;
      // define list pointers and initialize lists
      Node3D* nodes3D = new Node3D[length]();
      Node2D* nodes2D = new Node2D[width * height]();


      // ________________________
      // retrieving goal position
      float x = goal.pose.position.x / constants::cellSize;
      float y = goal.pose.position.y / constants::cellSize;
      float t = tf::getYaw(goal.pose.orientation);
      // set theta to a value (0,2PI]
      t = helper::normalizeHeadingRad(t);
      const Node3D nGoal(x, y, t, 0, 0, nullptr);
      // __________
      // DEBUG GOAL
      //      const Node3D nGoal(1, 1, M_PI, 0, 0, nullptr);


      // _________________________
      // retrieving start position
      x = start.pose.pose.position.x / constants::cellSize;
      y = start.pose.pose.position.y / constants::cellSize;
      t = tf::getYaw(start.pose.pose.orientation);
      // set theta to a value (0,2PI]
      t = helper::normalizeHeadingRad(t);
      Node3D nStart(x, y, t, 0, 0, nullptr);
      // ___________
      // DEBUG START
      //      Node3D nStart(1, 15, 0, 0, 0, nullptr);


      // ___________________________
      // START AND TIME THE PLANNING
      ros::Time t0 = ros::Time::now();

      // CLEAR THE VISUALIZATION
      visualization.clear();
      // FIND THE PATH
      Algorithm hybridAStar;
      //      Node3D* nSolution = Node3D::aStar();
      Node3D* nSolution = hybridAStar.findPath3D(nStart, nGoal, nodes3D, nodes2D, grid, collisionLookup, dubinsLookup, visualization);
      // CLEAR THE PATH
      path.clear();
      // TRACE THE PATH
      path.tracePath(nSolution);
      ros::Time t1 = ros::Time::now();
      ros::Duration d(t1 - t0);
      std::cout << "TIME in ms: " << d * 1000 << std::endl;

      // _________________________________
      // PUBLISH THE RESULTS OF THE SEARCH
      path.publishPath();
      path.publishPathNodes();
      path.publishPathVehicles();
      visualization.publishNode3DCosts(nodes3D, width, height, depth);
      visualization.publishNode2DCosts(nodes2D, width, height);

      delete [] nodes3D;
      delete [] nodes2D;

    } else {
      std::cout << "missing goal or start" << std::endl;
    }
  }
 private:
  ros::NodeHandle n;
  // publisher
  ros::Publisher pubStart;
  // subscriber
  ros::Subscriber subMap;
  ros::Subscriber subGoal;
  ros::Subscriber subStart;
  // tf listener
  tf::TransformListener listener;
  tf::StampedTransform transform;
  // PATH TRACING
  Path path;
  // ADDITIONAL VISUALIZATION
  Visualize visualization;
  // general pointer
  nav_msgs::OccupancyGrid::Ptr grid;
  bool validStart = false;
  geometry_msgs::PoseWithCovarianceStamped start;
  bool validGoal = false;
  geometry_msgs::PoseStamped goal;
  constants::config collisionLookup[constants::headings * constants::positions];
  float* dubinsLookup = new float [constants::headings * constants::headings * constants::dubinsWidth * constants::dubinsWidth];
};

#endif // SUBSCRIBEANDPUBLISH



