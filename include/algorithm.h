#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>
#include <rclcpp/rclcpp.hpp>

typedef ompl::base::SE2StateSpace::StateType State;

#include "node3d.h"
#include "node2d.h"
#include "visualize.h"
#include "collisiondetection.h"

namespace HybridAStar {
class Node3D;
class Node2D;
class Visualize;

namespace Algorithm {
Node3D* hybridAStar(Node3D& start,
                    const Node3D& goal,
                    Node3D* nodes3D,
                    Node2D* nodes2D,
                    int width,
                    int height,
                    CollisionDetection& configurationSpace,
                    float* dubinsLookup,
                    Visualize& visualization,
                    rclcpp::Node::SharedPtr n);
}
}
#endif // ALGORITHM_H
