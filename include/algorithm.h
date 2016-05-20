#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>

typedef ompl::base::SE2StateSpace::StateType State;

#include "node3d.h"
#include "node2d.h"
#include "visualize.h"
#include "collisiondetection.h"

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
                      int width,
                      int height,
                      CollisionDetection& configurationSpace,
                      float* dubinsLookup,
                      Visualize& visualization);

//  // A* ALGORITHM
//  float aStar(Node2D& start,
//              Node2D& goal,
//              const nav_msgs::OccupancyGrid::ConstPtr& grid,
//              Node2D* nodes2D,
//              Visualize& visualization);

//  // DUBINS SHOT
//  Node3D* dubinsShot(const Node3D& goal, CollisionDetection configurationSpace) const;

//  // UPDATE Node3D H value
//  void updateH(const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, CollisionDetection configurationSpace, Visualize& visualization);

};
}
#endif // ALGORITHM_H
