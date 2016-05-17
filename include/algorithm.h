#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "node3d.h"
#include "node2d.h"
#include "visualize.h"

namespace HybridAStar {
class Algorithm {
 public:
  // CONSTRUCTOR
  Algorithm() {}

  // HYBRID A* ALGORITHM
  static Node3D* findPath3D(Node3D& start,
                            const Node3D& goal,
                            Node3D* nodes3D,
                            Node2D* nodes2D,
                            const nav_msgs::OccupancyGrid::ConstPtr& grid,
                            Constants::config* collisionLookup,
                            float* dubinsLookup,
                            HybridAStar::Visualize& visualization);
};
}
#endif // ALGORITHM_H
