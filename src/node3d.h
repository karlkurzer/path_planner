#ifndef NODE3D_H
#define NODE3D_H

#include <cmath>

#include <nav_msgs/OccupancyGrid.h>

#include "dubins.h"
#include "node2d.h"

struct cfg;

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

    // GETTER METHODS
    float getX() const { return x; }
    float getY() const { return y; }
    float getT() const { return t; }
    float getG() const { return g; }
    float getH() const { return h; }
    float getC() const { return g + h; }
    int getIdx(int width, int height) const {return trunc(t / 5) * width * height + trunc(y) * width + trunc(x);}
    Node3D* getPred() const { return pred; }

    // SETTER METHODS
    void setX(const float& x) { this->x = x; }
    void setY(const float& y) { this->y = y; }
    void setT(const float& t) { this->t = t; }
    void setG(const float& g) { this->g = g; }
    void setH(const float& h) { this->h = h; }
    void setPred(Node3D* pred) { this->pred = pred; }

    // UPDATE METHODS
    // from start
    void updateG(const Node3D& pred) { g += movementCost(pred); }
    // to goal
    void updateH(const Node3D& goal, const nav_msgs::OccupancyGrid::ConstPtr& grid, float costGoal[]) { h = costToGo(goal, grid, costGoal); }

    // COST CALCULATION
    // cost for movement, g
    float movementCost(const Node3D& pred) const;
    // cost to go, dubins path or 2D A*
    float costToGo(const Node3D& goal, const nav_msgs::OccupancyGrid::ConstPtr& oGrid,
                   float costGoal[]) const;

    //  aStar algorithm
    //  static Node3D* aStar(Node3D& start, const Node3D& goal, const nav_msgs::OccupancyGrid::ConstPtr& oGrid);
    static Node3D* aStar(Node3D& start, const Node3D& goal,
                         const nav_msgs::OccupancyGrid::ConstPtr& oGrid, int width, int height, int depth, int length,
                         bool* open, bool* closed, float* cost, float* costToGo, float* costGoal);

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

////###################################################
////                                  CONST DECLARATION
////###################################################
//    HEADING => 0 - 359 degrees, 0 being north pointing towards negative X
//    X-COORDINATE => designating the length of the grid/world
//    Y-COORDINATE => designating the width of the grid/world

#endif // NODE3D_H
