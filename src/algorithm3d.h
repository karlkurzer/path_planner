#ifndef ALGORITHM3D_H
#define ALGORITHM3D_H

#include <nav_msgs/OccupancyGrid.h>

#include "node3d.h"

class Algorithm3D {
 public:
  Algorithm3D();

  static search(Node3D& start, const Node3D& goal,
                const nav_msgs::OccupancyGrid::ConstPtr& oGrid) {
  }
};

#endif // ALGORITHM3D_H
