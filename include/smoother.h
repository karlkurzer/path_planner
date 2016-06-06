#ifndef SMOOTHER_H
#define SMOOTHER_H

#include <cmath>
#include <vector>

#include "dynamicvoronoi.h"
#include "node3d.h"
#include "vector2d.h"
#include "helper.h"
#include "constants.h"
namespace HybridAStar {
/*!
   \brief This class takes a path object and smoothes the nodes of the path.

   It also uses the Voronoi diagram as well as the configuration space.
*/
class Smoother {
 public:
  Smoother();

  /*!
     \brief obstacleCost

     wObstacle * SUM penaltyFunction * (|x_i-o_i|-dMax)

     dMax is the road width
  */
  void obstacleTerm();

  /*!
     \brief curvatureCost


  */
  Vector2D curvatureTerm(Vector2D xi0, Vector2D xi1, Vector2D xi2);

  /*!
     \brief smoothnessCost

  */
  Vector2D smoothnessTerm(Vector2D xi0, Vector2D xi1, Vector2D xi2);

  /*!
     \brief voronoiCost - trade off between path length and closeness to obstacles

     alpha > 0 = falloff rate
     dObs(x,y) = distance to nearest obstacle
     dEge(x,y) = distance to nearest edge of the GVD
     dObsMax   = maximum distance for the cost to be applicable

     wVoronoi * SUM voronoiCost

  */
  void voronoiTerm();

  /*!
     \brief This function takes a path consisting of nodes and attempts to iteratively smooth the same using gradient descent.

     x_i = sequence of nodes
     o_i = the location of the obstacle nearest to the node x_i
     delta_x_i = x_i - x_(i-1) = the displacement vector at the vertex
     delta_theta_i = |tan^-1 (delta_y_(i+1)/delta_x_(i+1)) - tan^-1 (delta_y_(i)/delta_x_(i))| = the change in tangential angle at the vertex



     During the different interations the following cost are being calculated
     obstacleCost
     curvatureCost
     smoothnessCost
     voronoiCost
  */
  void smoothPath();

  /*!
     \brief Given a node pointer the path to the root node will be traced recursively
     \param node a 3D node, usually the goal node
     \param i a parameter for counting the number of nodes
  */
  void tracePath(const Node3D* node, int i = 0, std::vector<Node3D> path = std::vector<Node3D>());
  /// returns the path
  std::vector<Node3D> getPath() {return path;}

 private:
  /// maximum possible curvature of the non-holonomic vehicle
  float kappaMax = 1.f / (Constants::r * 1.1);
  /// maximum distance to obstacles that is penalized
  float obsDMax = Constants::minRoadWidth;
  /// maximum distance for obstacles to influence the voronoi field
  float vorObsDMax = Constants::minRoadWidth;
  /// falloff rate for the voronoi field
  float alpha = 1;
  /// weight for the obstacle term
  float wObstacle = 1;
  /// weight for the voronoi term
  float wVoronoi = 1;
  /// weight for the curvature term
  float wCurvature = 0.01;
  /// weight for the smoothness term
  float wSmoothness = 0.025;
  /// path to be smoothed
  std::vector<Node3D> path;
};
}
#endif // SMOOTHER_H
