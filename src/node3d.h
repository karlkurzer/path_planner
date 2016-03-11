#ifndef NODE3D_H
#define NODE3D_H

#include <cmath>

#include <nav_msgs/OccupancyGrid.h>

#include "dubins.h"
#include "node2d.h"
#include "constants.h"
#include "helper.h"
#include "visualize.h"


class Node3D {
 public:

  // CONSTRUCTOR
  Node3D(float x, float y, float t, float g, float h, Node3D* pred) {
    this->x = x;
    this->y = y;
    this->t = t;
    this->g = g;
    this->h = h;
    this->pred = pred;
    this->o = false;
    this->c = false;
  }
  // overloaded default constructor
  Node3D() {
    Node3D(0, 0, 0, 0, 0, nullptr);
  }
  // GETTER METHODS
  float getX() const { return x; }
  float getY() const { return y; }
  float getT() const { return t; }
  float getG() const { return g; }
  float getH() const { return h; }
  float getC() const { return g + h; }
  bool  isOpen() const { return o; }
  bool  isClosed() const { return c; }
  Node3D* getPred() const { return pred; }

  // SETTER METHODS
  void setX(const float& x) { this->x = x; }
  void setY(const float& y) { this->y = y; }
  void setT(const float& t) { this->t = t; }
  void setG(const float& g) { this->g = g; }
  void setH(const float& h) { this->h = h; }
  int setI(int width, int height) { this->i = (int)(t / constants::deltaHeadingRad) * width * height + (int)(y) * width + (int)(x); return i;}
  void open() { o = true; c = false;}
  void close() { c = true; o = false; }
  void setPred(Node3D* pred) { this->pred = pred; }

  // UPDATE METHODS
  // from start
  void updateG();
  // to goal
  void updateH(const Node3D& goal, const nav_msgs::OccupancyGrid::ConstPtr& grid, float* cost2d, float* dubinsLookup);

  // DUBINS SHOT
  Node3D* dubinsShot(const Node3D& goal, const nav_msgs::OccupancyGrid::ConstPtr& grid, constants::config* collisionLookup) const;

  // GRID CHECKING
  bool isOnGrid(const int width, const int height) const;

  // COLLISION CHECKING
  bool isTraversable(const nav_msgs::OccupancyGrid::ConstPtr& grid, constants::config* collisionLookup) const;

  // SUCCESSOR CREATION
  Node3D* createSuccessor(const int i) const;

  //  HYBRID A* ALGORITHM
  static Node3D* aStar(Node3D& start, const Node3D& goal, Node3D* nodes, float* cost2d,
                       const nav_msgs::OccupancyGrid::ConstPtr& grid, constants::config* collisionLookup, float* dubinsLookup);


  // CONSTANT VALUES
  // possible directions
  static const int dir;
  // possible movements
  static const float dx[];
  static const float dy[];
  static const float dt[];

 private:
  // x = position (length), y = position (width), t = heading, g = cost, h = cost to go, i = index, pred = pointer to predecessor node
  float x;
  float y;
  float t;
  float g;
  float h;
  int i;
  bool o;
  bool c;
  Node3D* pred;
};

#endif // NODE3D_H
