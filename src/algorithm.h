#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "node3d.h"
#include "node2d.h"
#include "visualize.h"


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
                            constants::config* collisionLookup,
                            float* dubinsLookup,
                            Visualize& visualization);
};

#endif // ALGORITHM_H
