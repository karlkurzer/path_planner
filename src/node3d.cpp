#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>

typedef ompl::base::SE2StateSpace::StateType State;

#include "node3d.h"

// CONSTANT VALUES
// possible directions
const int Node3D::dir = 3;
// possible movements
//const float Node3D::dy[] = { 0,        -0.032869,  0.032869};
//const float Node3D::dx[] = { 0.62832,   0.62717,   0.62717};
//const float Node3D::dt[] = { 0,         0.10472,   -0.10472};

// R = 6, 6.75 DEG
const float Node3D::dy[] = { 0,        -0.0415893,  0.0415893};
const float Node3D::dx[] = { 0.7068582,   0.705224,   0.705224};
const float Node3D::dt[] = { 0,         0.1178097,   -0.1178097};

// R = 3, 6.75 DEG
//const float Node3D::dy[] = { 0,        -0.0207946, 0.0207946};
//const float Node3D::dx[] = { 0.35342917352,   0.352612,  0.352612};
//const float Node3D::dt[] = { 0,         0.11780972451,   -0.11780972451};

//const float Node3D::dy[] = { 0,       -0.16578, 0.16578};
//const float Node3D::dx[] = { 1.41372, 1.40067, 1.40067};
//const float Node3D::dt[] = { 0,       0.2356194,   -0.2356194};

//###################################################
//                                         IS ON GRID
//###################################################
bool Node3D::isOnGrid(const int width, const int height) const {
  return x >= 0 && x < width && y >= 0 && y < height && (int)(t / constants::deltaHeadingRad) >= 0 && (int)(t / constants::deltaHeadingRad) < constants::headings;
}


//###################################################
//                                        IS IN RANGE
//###################################################
bool Node3D::isInRange(const Node3D& goal) const {
  float dx = std::abs(x - goal.x);
  float dy = std::abs(y - goal.y);
  return (dx * dx) + (dy * dy) < constants::dubinsShotDistance;
}

//###################################################
//                                   CREATE SUCCESSOR
//###################################################
Node3D* Node3D::createSuccessor(const int i) {
  float xSucc;
  float ySucc;
  float tSucc;

  // calculate successor positions forward
  if (i < 3) {
    xSucc = x + dx[i] * cos(t) - dy[i] * sin(t);
    ySucc = y + dx[i] * sin(t) + dy[i] * cos(t);
    tSucc = helper::normalizeHeadingRad(t + dt[i]);
  }
  // backwards
  else {
    xSucc = x - dx[i - 3] * cos(t) - dy[i - 3] * sin(t);
    ySucc = y - dx[i - 3] * sin(t) + dy[i - 3] * cos(t);
    tSucc = helper::normalizeHeadingRad(t + dt[i - 3] + M_PI);
  }

  return new Node3D(xSucc, ySucc, tSucc, g, 0, this, i);
}


//###################################################
//                                      MOVEMENT COST
//###################################################
void Node3D::updateG() {
  // penalize turning
  if (prim != 0) {
    if (pred->prim != prim) {
      g += dx[0] * constants::penaltyTurning * constants::penaltyTurning;
    } else {
      g += dx[0] * constants::penaltyTurning;
    }
  } else  {
    g += dx[0];
  }
}


//###################################################
//                                         COST TO GO
//###################################################
void Node3D::updateH(const Node3D& goal, const nav_msgs::OccupancyGrid::ConstPtr& grid, Node2D* nodes2D, float* dubinsLookup, Visualize& visualization) {
  float dubinsCost = 0;
  float reedsSheppCost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;

  // if dubins heuristic is activated calculate the shortest path
  // constrained without obstacles
  if (constants::dubins) {
    int uX = std::abs((int)goal.x - (int)x);
    int uY = std::abs((int)goal.y - (int)y);


    // if the lookup table flag is set and the vehicle is in the lookup area
    if (constants::dubinsLookup && uX < constants::dubinsWidth - 1 && uY < constants::dubinsWidth - 1) {
      int X = (int)goal.x - (int)x;
      int Y = (int)goal.y - (int)y;
      int h0;
      int h1;

      // mirror on x axis
      if (X >= 0 && Y <= 0) {
        h0 = (int)(helper::normalizeHeadingRad(M_PI_2 - t) / constants::deltaHeadingRad);
        h1 = (int)(helper::normalizeHeadingRad(M_PI_2 - goal.t) / constants::deltaHeadingRad);
      }
      // mirror on y axis
      else if (X <= 0 && Y >= 0) {
        h0 = (int)(helper::normalizeHeadingRad(M_PI_2 - t) / constants::deltaHeadingRad);
        h1 = (int)(helper::normalizeHeadingRad(M_PI_2 - goal.t) / constants::deltaHeadingRad);

      }
      // mirror on xy axis
      else if (X <= 0 && Y <= 0) {
        h0 = (int)(helper::normalizeHeadingRad(M_PI - t) / constants::deltaHeadingRad);
        h1 = (int)(helper::normalizeHeadingRad(M_PI - goal.t) / constants::deltaHeadingRad);

      } else {
        h0 = (int)(t / constants::deltaHeadingRad);
        h1 = (int)(goal.t / constants::deltaHeadingRad);
      }

      dubinsCost = dubinsLookup[uX * constants::dubinsWidth * constants::headings * constants::headings
                                + uY *  constants::headings * constants::headings
                                + h0 * constants::headings
                                + h1];
    } else { /*if (constants::dubinsShot && std::abs(x - goal.x) >= 10 && std::abs(y - goal.y) >= 10)*/
//      // start
//      double q0[] = { x, y, t};
//      // goal
//      double q1[] = { goal.x, goal.y, goal.t};
//      DubinsPath dubinsPath;
//      dubins_init(q0, q1, constants::r, &dubinsPath);
//      dubinsCost = dubins_path_length(&dubinsPath);

      ompl::base::DubinsStateSpace dubinsPath(constants::r);
      State* dbStart = (State*)dubinsPath.allocState();
      State* dbEnd = (State*)dubinsPath.allocState();
      dbStart->setXY(x, y);
      dbStart->setYaw(t);
      dbEnd->setXY(goal.x, goal.y);
      dbEnd->setYaw(goal.t);
      dubinsCost = dubinsPath.distance(dbStart, dbEnd);
    }
  }

  // if reversing is active use a
  if (constants::reverse && !constants::dubins) {
    ompl::base::ReedsSheppStateSpace reedsSheppPath(constants::r);
    State* rsStart = (State*)reedsSheppPath.allocState();
    State* rsEnd = (State*)reedsSheppPath.allocState();
    rsStart->setXY(x, y);
    rsStart->setYaw(t);
    rsEnd->setXY(goal.x, goal.y);
    rsEnd->setYaw(goal.t);
    reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
  }

  // if twoD heuristic is activated determine shortest path
  // unconstrained with obstacles
  if (constants::twoD && !nodes2D[(int)y * grid->info.width + (int)x].isDiscovered()) {
    // create a 2d start node
    Node2D start2d(x, y, 0, 0, nullptr);
    // create a 2d goal node
    Node2D goal2d(goal.x, goal.y, 0, 0, nullptr);
    // run 2d astar and return the cost of the cheapest path for that node
    nodes2D[(int)y * grid->info.width + (int)x].setG(Node2D::aStar(goal2d, start2d, grid, nodes2D, visualization));
  }

  if (constants::twoD) {
    // offset for same node in cell
    twoDoffset = sqrt(((x - (long)x) - (goal.x - (long)goal.x)) * ((x - (long)x) - (goal.x - (long)goal.x)) +
                      ((y - (long)y) - (goal.y - (long)goal.y)) * ((y - (long)y) - (goal.y - (long)goal.y)));
    twoDCost = nodes2D[(int)y * grid->info.width + (int)x].getG() - twoDoffset;

  }

  // return the maximum of the heuristics, making the heuristic admissable
  h = std::max(reedsSheppCost, std::max(dubinsCost, twoDCost));
}

//###################################################
//                                 COLLISION CHECKING
//###################################################
bool Node3D::isTraversable(const nav_msgs::OccupancyGrid::ConstPtr& grid, constants::config* collisionLookup) const {
  int X = (int)x;
  int Y = (int)y;
  int iX = (int)((x - (long)x) * constants::positionResolution);
  iX = iX > 0 ? iX : 0;
  int iY = (int)((y - (long)y) * constants::positionResolution);
  iY = iY > 0 ? iY : 0;
  int iT = (int)(t / constants::deltaHeadingRad);
  int idx = iY * constants::positionResolution * constants::headings + iX * constants::headings + iT;
  int cX;
  int cY;

  for (int i = 0; i < collisionLookup[idx].length; ++i) {
    cX = (X + collisionLookup[idx].pos[i].x);
    cY = (Y + collisionLookup[idx].pos[i].y);

    // make sure the configuration coordinates are actually on the grid
    if (cX >= 0 && (unsigned int)cX < grid->info.width && cY >= 0 && (unsigned int)cY < grid->info.height) {
      if (grid->data[cY * grid->info.width + cX]) {
        return false;
      }
    }
  }

  return true;
}


//###################################################
//                                        DUBINS SHOT
//###################################################
Node3D* Node3D::dubinsShot(const Node3D& goal, const nav_msgs::OccupancyGrid::ConstPtr& grid, constants::config* collisionLookup) const {
  // start
  double q0[] = { x, y, t };
  // goal
  double q1[] = { goal.x, goal.y, goal.t };
  // initialize the path
  DubinsPath path;
  // calculate the path
  dubins_init(q0, q1, constants::r, &path);

  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);

  Node3D* dubinsNodes = new Node3D [(int)(length / constants::dubinsStepSize) + 1];

  while (x <  length) {
    double q[3];
    dubins_path_sample(&path, x, q);
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(helper::normalizeHeadingRad(q[2]));

    // collision check
    if (dubinsNodes[i].isTraversable(grid, collisionLookup)) {

      // set the predecessor to the previous step
      if (i > 0) {
        dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i].setPred(this);
      }

      if (&dubinsNodes[i] == dubinsNodes[i].pred) {
        std::cout << "looping shot";
      }

      x += constants::dubinsStepSize;
      i++;
    } else {
      //      std::cout << "Dubins shot collided, discarding the path" << "\n";
      // delete all nodes
      delete [] dubinsNodes;
      return nullptr;
    }
  }

  //  std::cout << "Dubins shot connected, returning the path" << "\n";
  return &dubinsNodes[i - 1];
}

//###################################################
//                                 3D NODE COMPARISON
//###################################################
bool Node3D::operator == (const Node3D& rhs) const {
  return (int)x == (int)rhs.x &&
         (int)y == (int)rhs.y &&
         (std::abs(t - rhs.t) <= constants::deltaHeadingRad ||
          std::abs(t - rhs.t) >= constants::deltaHeadingNegRad);
}
