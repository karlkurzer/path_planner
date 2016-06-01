#ifndef SMOOTHER_H
#define SMOOTHER_H

#include <cmath>

#include "dynamicvoronoi.h"
#include "constants.h"
namespace HybridAStar {
/*!
   \brief This class takes a path object and smoothes the nodes of the path.

   It also uses the Voronoi diagram as well as the configuration space.
*/
class Smoother {
 public:
  Smoother();
  ~Smoother();

  /*!
     \brief obstacleCost

     wObstacle * SUM penaltyFunction * (|x_i-o_i|-dMax)

     dMax is the road width
  */
  void obstacleTerm();

  /*!
     \brief curvatureCost


  */
  void curvatureTerm();

  /*!
     \brief smoothnessCost

  */
  void smoothnessTerm();

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

 private:
  /// maximum possible curvature of the non-holonomic vehicle
  float kMax = 1.f / (Constants::r * 1.f);
  /// maximum distance to obstacles that is penalized
  float obsDMax = Constants::minRoadWidth;
  /// maximum distance for obstacles to influence the voronoi field
  float vorObsDMax = Constants::minRoadWidth;
  /// falloff rate for the voronoi field
  float alpha = 0;
  /// weight for the obstacle term
  float wObstacle = 0;
  /// weight for the collision term
  float wCollision = 0;
  /// weight for the curvature term
  float wCurvature = 0;
  /// weight for the smoothness term
  float wSmoothness = 0;
};
}
#endif // SMOOTHER_H
