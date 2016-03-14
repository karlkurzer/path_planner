#include <functional>
#include <queue>
#include <vector>

#include "node3d.h"
#include "visualize.h"

// CONSTANT VALUES
// possible directions
const int Node3D::dir = 3;
// possible movements
const float Node3D::dy[] = { 0,        -0.032869,  0.032869};
const float Node3D::dx[] = { 0.62832,   0.62717,   0.62717};
const float Node3D::dt[] = { 0,         0.10472,   -0.10472};

//const float Node3D::dx[] = { 0,       -0.07387, 0.07387};
//const float Node3D::dy[] = { 0.94248, 0.938607, 0.938607};
//const float Node3D::dt[] = { 0,       9,   -9};

//const float Node3D::dx[] = { 0,       -0.16578, 0.16578};
//const float Node3D::dy[] = { 1.41372, 1.40067, 1.40067};
//const float Node3D::dt[] = { 0,       13.5,   -13.5};

//###################################################
//                                         IS ON GRID
//###################################################
bool Node3D::isOnGrid(const int width, const int height) const {
  return x >= 0 && x < width && y >= 0 && y < height && (int)(t / constants::deltaHeadingRad) >= 0 && (int)(t / constants::deltaHeadingRad) < constants::headings;
}


//###################################################
//                                   CREATE SUCCESSOR
//###################################################
Node3D* Node3D::createSuccessor(const int i) const {
  // calculate successor positions
  float xSucc = x + dx[i] * cos(t) - dy[i] * sin(t);
  float ySucc = y + dx[i] * sin(t) + dy[i] * cos(t);
  float tSucc = helper::normalizeHeadingRad(t + dt[i]);
  float g = pred != nullptr ? pred->getG() : 0;
  return new Node3D(xSucc, ySucc, tSucc, g, 0, nullptr);
}


//###################################################
//                                      MOVEMENT COST
//###################################################
void Node3D::updateG() {
  float predT = pred != nullptr ? pred->getT() : t;

  // penalize turning
  if ((int)t != t) {
    g += dx[0] * constants::penaltyTurning;
  } else  {
    g += dx[0];
  }

}


//###################################################
//                                         COST TO GO
//###################################################
void Node3D::updateH(const Node3D& goal, const nav_msgs::OccupancyGrid::ConstPtr& grid, float* cost2d, float* dubinsLookup) {
  float dubinsCost = 0;
  float euclideanCost = 0;

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
      // start
      double q0[] = { x, y, t};
      // goal
      double q1[] = { goal.x, goal.y, goal.t};
      DubinsPath path;
      dubins_init(q0, q1, constants::r, &path);
      dubinsCost = dubins_path_length(&path);
    }
  }

  // if twoD heuristic is activated determine shortest path
  // unconstrained with obstacles
  if (constants::twoD && cost2d[(int)y * grid->info.width + (int)x] == 0) {
    // create a 2d start node
    Node2D start2d(x, y, 0, 0, nullptr);
    // create a 2d goal node
    Node2D goal2d(goal.x, goal.y, 0, 0, nullptr);
    // run 2d astar and return the cost of the cheapest path for that node
    cost2d[(int)y * grid->info.width + (int)x] = Node2D::aStar(goal2d, start2d, grid, cost2d);
  }

  // else calculate the euclidean distance
  euclideanCost = sqrt((x - goal.x) * (x - goal.x) + (y - goal.y) * (y - goal.y));
  // return the maximum of the heuristics, making the heuristic admissable
  h = std::max(euclideanCost, std::max(dubinsCost, cost2d[(int)y * grid->info.width + (int)x]));
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
    if (cX >= 0 && cX < grid->info.width && cY >= 0 && cY < grid->info.height) {
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
        dubinsNodes[i].setPred(pred);
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
struct CompareNodes : public
  std::binary_function<Node3D*, Node3D*, bool> {
  bool operator()(const Node3D* lhs, const Node3D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
};

bool operator == (const Node3D& lhs, const Node3D& rhs) {
  return (int)lhs.getX() == (int)rhs.getX() &&
         (int)lhs.getY() == (int)rhs.getY() &&
         std::abs(lhs.getT() - rhs.getT()) <= constants::deltaHeadingRad;
}

//###################################################
//                                 				3D A*
//###################################################
Node3D* Node3D::aStar(Node3D& start,
                      const Node3D& goal,
                      Node3D* nodes,
                      float* cost2d,
                      const nav_msgs::OccupancyGrid::ConstPtr& grid,
                      constants::config* collisionLookup,
                      float* dubinsLookup) {

  Visualize visualization;
  ros::Duration d(0.1);

  // PREDECESSOR AND SUCCESSOR POSITION
  int iPred, iSucc;
  int width = grid->info.width;
  int height = grid->info.height;
  float newG;

  // OPEN LIST
  std::priority_queue<Node3D*, std::vector<Node3D*>, CompareNodes> O;
  // update h value
  start.updateH(goal, grid, cost2d, dubinsLookup);
  // mark start as open
  start.open();
  // push on priority queue aka open list
  O.push(&start);
  iPred = start.setI(width, height);
  nodes[iPred] = start;

  // create new node pointer
  Node3D* nPred;
  Node3D* nSucc;



  // continue until O empty
  while (!O.empty()) {
    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->setI(width, height);
    // RViz mark current node
    // publish the new node
    visualization.publishNode3D(*nPred);
    d.sleep();

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes[iPred].isOpen()) {
      // add node to closed list
      nodes[iPred].close();
      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST
      if (*nPred == goal) {
        return nPred;
      }

      // ____________________
      // CONTINUE WITH SEARCH
      else {
        // _______________________
        // SEARCH WITH DUBINS SHOT
        if (constants::dubinsShot && std::abs(nPred->getX() - goal.x) < 10 && std::abs(nPred->getY() - goal.y) < 10) {
          nSucc = nPred->dubinsShot(goal, grid, collisionLookup);

          if (nSucc != nullptr) { return nSucc; }
        }

        // ______________________________
        // SEARCH WITH FORWARD SIMULATION
        for (int i = 0; i < 3; i++) {

          // create possible successor
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc->setI(width, height);

          // ensure successor is on grid and traversable
          if (nSucc->isOnGrid(width, height) && nSucc->isTraversable(grid, collisionLookup)) {

            // ensure successor is not on closed list or it has the same index as the predecessor
            if (!nodes[iSucc].isClosed() || iPred == iSucc) {

              // calculate new G value
              nSucc->updateG();
              newG = nSucc->getG();

              // if successor not on open list or found a shorter way to the cell
              if (!nodes[iSucc].isOpen() || newG < nodes[iSucc].getG() || iPred == iSucc) {

                // calculate heuristic
                nSucc->updateH(goal, grid, cost2d, dubinsLookup);

                // if successor is in the same cell set predecessor to predecessor of predecessor
                if (iPred == iSucc && nSucc->getH() < nPred->getH()) {
                  nSucc->setPred(nPred->getPred());
                  // remove from closed list so that it can be expanded again
                  //                  delete closed[nPred->getI()];
                  //                  closed[nPred->getI()] = nullptr;

                }
                //set predecessor to predecessor
                else {
                  nSucc->setPred(nPred);
                }

                // put successor on open list
                nSucc->open();
                nodes[iSucc] = *nSucc;
                O.push(&nodes[iSucc]);
                delete nSucc;
              } else { delete nSucc; }
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  if (O.empty()) {
    return nullptr;
  }
}
