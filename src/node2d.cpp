#include "node2d.h"

using namespace HybridAStar;

// initialize static data
geometry_msgs::Pose Node2D::mp;
float Node2D::res = 1.0;

// possible directions
const int Node2D::dir = 8;
// possible movements
const int Node2D::dx[] = { -1, -1, 0, 1, 1, 1, 0, -1 };
const int Node2D::dy[] = { 0, 1, 1, 1, 0, -1, -1, -1 };

//###################################################
//                                         IS ON GRID
//###################################################
bool Node2D::isOnGrid(const int width, const int height) const {
  return  (y > mp.position.y && y <= (res * height + mp.position.y)) && (x > mp.position.x && x <= (res * width + mp.position.x));
}

//###################################################
//                                   CREATE SUCCESSOR
//###################################################
Node2D* Node2D::createSuccessor(const int i) {
  int xSucc = x + Node2D::dx[i];
  int ySucc = y + Node2D::dy[i];
  return new Node2D(xSucc, ySucc, g, 0, this);
}

//###################################################
//                                 2D NODE COMPARISON
//###################################################
bool Node2D::operator == (const Node2D& rhs) const {
  return x == rhs.x && y == rhs.y;
}
