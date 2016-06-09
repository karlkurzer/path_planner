#include "smoother.h"
using namespace HybridAStar;
//###################################################
//                                     CUSP DETECTION
//###################################################
inline bool isCusp(std::vector<Node3D> path, int i) {
  bool revim2 = path[i - 2].getPrim() > 3 ? true : false;
  bool revim1 = path[i - 1].getPrim() > 3 ? true : false;
  bool revi   = path[i].getPrim() > 3 ? true : false;
  bool revip1 = path[i + 1].getPrim() > 3 ? true : false;
  //  bool revip2 = path[i + 2].getPrim() > 3 ? true : false;

  if (revim2 != revim1 || revim1 != revi || revi != revip1) { return true; }

  return false;
}
//###################################################
//                                SMOOTHING ALGORITHM
//###################################################
void Smoother::smoothPath(DynamicVoronoi& voronoi) {
  // load the current voronoi diagram into the smoother
  this->voronoi = voronoi;
  this->width = voronoi.getSizeX();
  this->height = voronoi.getSizeY();
  // current number of iterations of the gradient descent smoother
  int iterations = 0;
  // the maximum iterations for the gd smoother
  int maxIterations = 500;
  // the lenght of the path in number of nodes
  int pathLength = 0;

  // path objects with all nodes oldPath the original, newPath the resulting smoothed path
  pathLength = path.size();
  std::vector<Node3D> newPath = path;

  // descent along the gradient untill the maximum number of iterations has been reached
  while (iterations < maxIterations) {

    // choose the first three nodes of the path
    for (int i = 2; i < pathLength - 2; ++i) {

      Vector2D xi0(newPath[i - 1].getX(), newPath[i - 1].getY());
      Vector2D xi1(newPath[i].getX(), newPath[i].getY());
      Vector2D xi2(newPath[i + 1].getX(), newPath[i + 1].getY());
      Vector2D correction;


      // the following points shall not be smoothed
      // keep these points fixed if they are a cusp point or adjacent to one
      if (isCusp(newPath, i)) { continue; }

      correction = correction - obstacleTerm(xi1);
      if (!isOnGrid(xi1 + correction)) { continue; }

      //      voronoiTerm();
      // ensure that it is on the grid
      correction = correction - smoothnessTerm(xi0, xi1, xi2);
      if (!isOnGrid(xi1 + correction)) { continue; }

      // ensure that it is on the grid
      correction = correction - curvatureTerm(xi0, xi1, xi2);
      if (!isOnGrid(xi1 + correction)) { continue; }

      // ensure that it is on the grid

      xi1 = xi1 + correction;
      newPath[i].setX(xi1.getX());
      newPath[i].setY(xi1.getY());
      Vector2D Dxi1 = xi1 - xi0;
      newPath[i - 1].setT(std::atan2(Dxi1.getY(), Dxi1.getX()));

    }

    iterations++;
  }

  path = newPath;
}

void Smoother::tracePath(const Node3D* node, int i, std::vector<Node3D> path) {
  if (node == nullptr) {
    this->path = path;
    return;
  }

  i++;
  path.push_back(*node);
  tracePath(node->getPred(), i, path);
}

//###################################################
//                                      OBSTACLE TERM
//###################################################
Vector2D Smoother::obstacleTerm(Vector2D xi) {
  Vector2D gradient;
  // the distance to the closest obstacle from the current node
  float obsDst = voronoi.getDistance(xi.getX(), xi.getY());
  // the vector determining where the obstacle is
  Vector2D obsVct(xi.getX() - voronoi.data[(int)xi.getX()][(int)xi.getY()].obstX,
                  xi.getY() - voronoi.data[(int)xi.getX()][(int)xi.getY()].obstY);

  // the closest obstacle is closer than desired correct the path for that
  if (obsDst < obsDMax) {
    return gradient = wObstacle * 2 * (obsDst - obsDMax) * obsVct / obsDst;
  }
  return gradient;
}

//###################################################
//                                       VORONOI TERM
//###################################################
Vector2D Smoother::voronoiTerm() {
  Vector2D gradient;
  //    alpha > 0 = falloff rate
  //    dObs(x,y) = distance to nearest obstacle
  //    dEge(x,y) = distance to nearest edge of the GVD
  //    dObsMax   = maximum distance for the cost to be applicable
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
      float PobsDst_Pxi = obsVct / obsDst;
      float PedgDst_Pxi = vorVct / edgDst;
      float PvorPtn_PedgDst = alpha * obsDst * std::pow(obsDst - vorObsDMax, 2) / (std::pow(vorObsDMax, 2)
                              * (obsDst + alpha) * std::pow(edgDst + obsDst, 2));

      float PvorPtn_PobsDst = (alpha * edgDst * (obsDst - vorObsDMax) * ((edgDst + 2 * vorObsDMax + alpha)
                               * obsDst + (vorObsDMax + 2 * alpha) * edgDst + alpha * vorObsDMax))
                              / (std::pow(vorObsDMax, 2) * std::pow(obsDst + alpha, 2) * std::pow(obsDst + edgDst, 2));
      gradient = wVoronoi * PvorPtn_PobsDst * PobsDst_Pxi + PvorPtn_PedgDst * PedgDst_Pxi;

    }
  }
}

//###################################################
//                                     CURVATURE TERM
//###################################################
Vector2D Smoother::curvatureTerm(Vector2D xi0, Vector2D xi1, Vector2D xi2) {
  Vector2D gradient;
  // the vectors between the nodes
  Vector2D Dxi1 = xi1 - xi0;
  Vector2D Dxi2 = xi2 - xi1;
  // orthogonal complements vector
  Vector2D p1, p2;

  // the distance of the vectors
  float absDxi1 = Dxi1.length();
  float absDxi2 = Dxi2.length();

  // ensure that the absolute values are not null
  if (absDxi1 > 0 && absDxi2 > 0) {
    // the angular change at the node
    float Dphi1 = std::acos(Helper::clamp(Dxi1.dot(Dxi2) / (absDxi1 * absDxi2), -1, 1));
    float kappa = Dphi1 / absDxi1;

    // if the curvature is smaller then the maximum do nothing
    if (kappa <= kappaMax) {
      Vector2D zeros;
      return zeros;
    } else {
      float absDxi1Inv = 1 / absDxi1;
      float PDphi1_PcosDphi1 = -1 / std::sqrt(1 - std::pow(std::cos(Dphi1), 2));
      float u = -absDxi1Inv * PDphi1_PcosDphi1;
      // calculate the p1 and p2 terms
      p1 = xi1.ort(-xi2) / (absDxi1 * absDxi2);
      p2 = -xi2.ort(xi1) / (absDxi1 * absDxi2);
      // calculate the last terms
      float cxi1 = Dphi1 / (absDxi1 * absDxi1);
      Vector2D ones(1, 1);
      Vector2D kxi1 = u * (-p1 - p2) - (cxi1 * ones);
      Vector2D kxi0 = u * p2 - (cxi1 * ones);
      Vector2D kxi2 = u * p1;

      // calculate the gradient
      gradient = wCurvature * (0.25 * kxi0 + 0.5 * kxi1 + 0.25 * kxi2);

      if (std::isnan(gradient.getX()) || std::isnan(gradient.getY())) {
        std::cout << "nan values in curvature term" << std::endl;
        Vector2D zeros;
        return zeros;
      }
      // return gradient of 0
      else {
        return gradient;
      }
    }
  }
  // return gradient of 0
  else {
    std::cout << "abs values not larger than 0" << std::endl;
    Vector2D zeros;
    return zeros;
  }
}

//###################################################
//                                    SMOOTHNESS TERM
//###################################################
Vector2D Smoother::smoothnessTerm(Vector2D xi0, Vector2D xi1, Vector2D xi2) {
  return wSmoothness * (-4 * xi2 + 8 * xi1 - 4 * xi0);
}

