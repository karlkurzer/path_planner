#include "smoother.h"

using namespace HybridAStar;
Smoother::Smoother() {

}

//###################################################
//                                SMOOTHING ALGORITHM
//###################################################
void Smoother::smoothPath() {
  int iterations = 0;
  int maxIterations = 1000;
  int pathLength = 0;

  // path objects with all nodes oldPath the original, newPath the resulting smoothed path
  // newPath = oldPath;
  // pathLength = oldPath.pathLength;
  //

  // descent along the gradient untill the maximum number of iterations has been reached
  while (iterations < maxIterations) {

    // choose the first three nodes of the path
    for (int i = 2; i < pathLength - 2; ++i) {

      // the following points shall not be smoothed
      // keep these points fixed if they are a cusp point or adjacent to one
      //          if (newPath[i - 2].Gear != newPath[i - 1].Gear) continue;
      //          if (newPath[i - 1].Gear != newPath[i].Gear) continue;
      //          if (newPath[i].Gear != newPath[i + 1].Gear) continue;
      //          if (newPath[i + 1].Gear != newPath[i + 2].Gear) continue;

      obstacleTerm();
      // ensure that it is on the grid
      voronoiTerm();
      // ensure that it is on the grid
      curvatureTerm();
      // ensure that it is on the grid
      smoothnessTerm();
      // ensure that it is on the grid
    }

    iterations++;
  }
}


//###################################################
//                                      OBSTACLE TERM
//###################################################
void Smoother::obstacleTerm() {
  float gradient;
  float obsDst;
  // the vector determining where the obstacle is
  float obsVct;

  //calculate the distance to the closest obstacle from the current node
  //obsDist =  voronoiDiagram.getDistance(node->getX(),node->getY())

  // the closest obstacle is closer than desired correct the path for that
  // negative to move away from the obstacle closest
  if (obsDst < obsDMax) {
    gradient = wObstacle * (obsDst - obsDMax) * obsVct / obsDst;
  }
}

//###################################################
//                                       VORONOI TERM
//###################################################
void Smoother::voronoiTerm() {
  // distance to the closest obstacle
  float obsDst;
  // distance to the closest voronoi edge
  float edgDst;
  // the vector determining where the obstacle is
  float obsVct;
  // the vector determining where the voronoi edge is
  float vorVct;
  //calculate the distance to the closest obstacle from the current node
  //obsDist =  voronoiDiagram.getDistance(node->getX(),node->getY())

  if (obsDst < vorObsDMax) {
    //calculate the distance to the closest GVD edge from the current node
    //vorDist =  voronoiDiagram.getDistance(node->getX(),node->getY())
    // the node is away from the optimal free space area
    if (edgDst > 0) {
      float DobsDst_Dxi = obsVct / obsDst;
      float DedgDst_Dxi = vorVct / edgDst;
      float DvorPtn_DedgDst = alpha * obsDst * std::pow(obsDst - vorObsDMax, 2) / (std::pow(vorObsDMax, 2)
                              * (obsDst + alpha) * std::pow(edgDst + obsDst, 2));

      float DvorPtn_DobsDst = (alpha * edgDst * (obsDst - vorObsDMax) * ((edgDst + 2 * vorObsDMax + alpha)
                               * obsDst + (vorObsDMax + 2 * alpha) * edgDst + alpha * vorObsDMax))
                              / (std::pow(vorObsDMax, 2) * std::pow(obsDst + alpha, 2) * std::pow(obsDst + edgDst, 2));

    }
  }
}

//###################################################
//                                     CURVATURE TERM
//###################################################
void Smoother::curvatureTerm() {

}

//###################################################
//                                    SMOOTHNESS TERM
//###################################################
void Smoother::smoothnessTerm() {

}

