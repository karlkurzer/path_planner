#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "node3d.h"
#include "node2d.h"
#include "visualize.h"

namespace HybridAStar {
class Node3D;
class Node2D;
class Visualize;

class Algorithm {
 public:
  // CONSTRUCTOR
  Algorithm() {}

  // HYBRID A* ALGORITHM
  static Node3D* hybridAStar(Node3D& start,
                             const Node3D& goal,
                             Node3D* nodes3D,
                             Node2D* nodes2D,
                             const nav_msgs::OccupancyGrid::ConstPtr& grid,
                             Constants::config* collisionLookup,
                             float* dubinsLookup,
                             Visualize& visualization);

  // A* ALGORITHM
  static float aStar(Node2D& start,
                     Node2D& goal,
                     const nav_msgs::OccupancyGrid::ConstPtr& grid,
                     Node2D* nodes2D,
                     Visualize& visualization);

};
}
#endif // ALGORITHM_H
