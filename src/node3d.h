#ifndef NODE3D_H
#define NODE3D_H

#include <cmath>

#include <nav_msgs/OccupancyGrid.h>

#include "dubins.h"
#include "node2d.h"
#include "constants.h"
#include "helper.h"


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
  }
  // overloaded default constructor
  Node3D() {
    Node3D(0, 0, 0, 0, 0, nullptr);
  }
  // GETTER METHODS
  inline float getX() const { return x; }
  inline float getY() const { return y; }
  inline float getT() const { return t; }
  inline float getG() const { return g; }
  inline float getH() const { return h; }
  inline float getC() const { return g + h; }
  inline int getIdx(int width, int height) const {return (int)(t / constants::deltaHeadingRad) * width * height + (int)(y) * width + (int)(x);}
  inline Node3D* getPred() const { return pred; }

  // SETTER METHODS
  inline void setX(const float& x) { this->x = x; }
  inline void setY(const float& y) { this->y = y; }
  inline void setT(const float& t) { this->t = t; }
  inline void setG(const float& g) { this->g = g; }
  inline void setH(const float& h) { this->h = h; }
  inline void setPred(Node3D* pred) { this->pred = pred; }

  // UPDATE METHODS
  // from start
  inline void updateG(const Node3D& pred) { g += movementCost(pred); }
  // to goal
  inline void updateH(const Node3D& goal, const nav_msgs::OccupancyGrid::ConstPtr& grid, float* cost2d, float* dubinsLookup)
  { h = costToGo(goal, grid, cost2d, dubinsLookup); }

  // COST CALCULATION
  // cost for movement, g
  inline float movementCost(const Node3D& pred) const;
  // cost to go, dubins path or 2D A*
  inline float costToGo(const Node3D& goal, const nav_msgs::OccupancyGrid::ConstPtr& oGrid, float* cost2d, float* dubinsLookup) const;

  // DUBINS SHOT
  inline Node3D* dubinsShot(const Node3D& goal, const nav_msgs::OccupancyGrid::ConstPtr& grid, constants::config* collisionLookup) const;

  //  HYBRID A* ALGORITHM
  static Node3D* aStar(Node3D& start, const Node3D& goal, bool* open, bool* closed, float* cost, float* costToGo, float* cost2d,
                       const nav_msgs::OccupancyGrid::ConstPtr& grid, constants::config* collisionLookup, float* dubinsLookup);

  static inline bool collisionChecking(const nav_msgs::OccupancyGrid::ConstPtr& grid, constants::config* collisionLookup, float x, float y, float t);

  // CONSTANT VALUES
  // possible directions
  static const int dir;
  // possible movements
  static const float dx[];
  static const float dy[];
  static const float dt[];

 private:
  // x = position (length), y = position (width), t = heading, g = cost, h = cost to go, pred = pointer to predecessor node
  float x;
  float y;
  float t;
  float g;
  float h;
  Node3D* pred;
};

#endif // NODE3D_H
