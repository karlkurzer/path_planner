#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "constants.h"
#include "lookup.h"
#include "node2d.h"
#include "node3d.h"

namespace HybridAStar {
namespace {
inline void getConfiguration(const Node2D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  t = 99;
}

inline void getConfiguration(const Node3D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  t = node->getT();
}
}
class CollisionDetection {
 public:
  CollisionDetection();

  template<typename T> bool isTraversable(const T* node) const {
    float cost = 0;
    float x;
    float y;
    float t;
    getConfiguration(node, x, y, t);

    if (t == 99) {
      return !grid->data[node->getIdx()];
    }

    if (true) {
      cost = configurationTest(x, y, t) ? 0 : 1;
    } else {
      cost = configurationCost(x, y, t);
    }

    return cost <= 0;
  }

  float configurationCost(float x, float y, float t) const {return 0;}

  bool configurationTest(float x, float y, float t) const;

  void updateGrid(nav_msgs::msg::OccupancyGrid::SharedPtr map) {grid = map;}

 private:
  nav_msgs::msg::OccupancyGrid::SharedPtr grid;
  Constants::config collisionLookup[Constants::headings * Constants::positions];
};
}
#endif // COLLISIONDETECTION_H
