#include <functional>
#include <queue>
#include <vector>

#include "node3d.h"

auto count2D = 0;

// CONSTANT VALUES
// possible directions
const int Node3D::dir = 3;
// possible movements
//const float Node3D::dx[] = { 0,         -0.018496,  0.018496};
//const float Node3D::dy[] = { 0.471239,  0.470755,   0.470755};
//const float Node3D::dt[] = { 0,         4.5,        -4.5};

const float Node3D::dx[] = { 0,       -0.07387, 0.07387};
const float Node3D::dy[] = { 0.94248, 0.938607, 0.938607};
const float Node3D::dt[] = { 0,       9,   -9};

//const float Node3D::dx[] = { 0,       -0.16578, 0.16578};
//const float Node3D::dy[] = { 1.41372, 1.40067, 1.40067};
//const float Node3D::dt[] = { 0,       13.5,   -13.5};

//###################################################
//                                      MOVEMENT COST
//###################################################
float Node3D::movementCost(const Node3D& pred) const {

  float distance = dy[0];
  // euclidean distance
  //  distance = sqrt((x - pred.x) * (x - pred.x) + (y - pred.y) * (y - pred.y));
  distance = dy[0];
  return distance;
}

//###################################################
//                                         COST TO GO
//###################################################
float Node3D::costToGo(const Node3D& goal,
                       const nav_msgs::OccupancyGrid::ConstPtr& grid,
                       float cost2d[]) const {
  const bool dubins = false;
  const bool twoD = false;
  float dubinsCost = 0;
  float euclideanCost = 0;

  // if dubins heuristic is activated calculate the shortest path
  // constrained without obstacles
  if (dubins) {
    // minimum turning radius
    const float r = 6;
    // start
    double q0[] = { x, y, (t + 90) / 180 * M_PI };
    // goal
    double q1[] = { goal.x, goal.y, (goal.t + 90) / 180 * M_PI };
    DubinsPath path;
    dubins_init(q0, q1, r, &path);
    dubinsCost = dubins_path_length(&path);
  }

  // if twoD heuristic is activated determine shortest path
  // unconstrained with obstacles
  if (twoD && cost2d[(int)y * grid->info.width + (int)x] == 0) {
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
  return std::max(euclideanCost, std::max(dubinsCost, cost2d[(int)y * grid->info.width + (int)x]));
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
  return trunc(lhs.getX()) == trunc(rhs.getX()) && trunc(lhs.getY()) == trunc(rhs.getY()) &&
         std::abs(std::abs(lhs.getT()) - std::abs(rhs.getT())) <= 5;
}

//###################################################
//                                 				3D A*
//###################################################
Node3D* Node3D::aStar(Node3D& start, const Node3D& goal,
                      const nav_msgs::OccupancyGrid::ConstPtr& grid, int width, int height, int depth, int length,
                      bool* open, bool* closed, float* cost, float* costToGo, float* cost2d) {

  // PREDECESSOR AND SUCCESSOR POSITION
  float x, y, t, xSucc, ySucc, tSucc;
  int idx = 0;
  int idxSucc = 0;


  // OPEN LIST
  std::priority_queue<Node3D*, std::vector<Node3D*>, CompareNodes> O;
  // update h value
  start.updateH(goal, grid, cost2d);
  // push on priority queue
  O.push(&start);
  // add node to open list with total estimated cost
  cost[start.getIdx(width, height)] = start.getC();
  // create new node pointer
  Node3D* nPred;

  // continue until O empty
  while (!O.empty()) {
    // pop node with lowest cost from priority queue
    nPred = O.top();
    x = nPred->getX();
    y = nPred->getY();
    t = nPred->getT();
    //    std::cout <<"Expanding\nx: " <<x <<"\ny: " <<y <<"\nt: " <<t <<std::endl;
    idx = nPred->getIdx(width, height);

    // lazy deletion of rewired node
    if (closed[idx] == true) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    } else if (closed[idx] == false) {
      // remove node from open list
      O.pop();
      open[idx] = false;
      // add node to closed list
      closed[idx] = true;

      // goal test
      if (*nPred == goal) {
        return nPred;
      }
      // continue with search
      else {
        // loop over possible successor nodes
        for (int i = 0; i < 3; i++) {

          // calculate successor positions
          xSucc = x + dx[i] * cos(t / 180 * M_PI) - dy[i] * sin(t / 180 * M_PI);
          ySucc = y + dx[i] * sin(t / 180 * M_PI) + dy[i] * cos(t / 180 * M_PI);
          tSucc = t + dt[i];

          // limit theta to 0-360
          if (tSucc >= 360)  { tSucc = tSucc - 360; }
          else if (tSucc < 0)  { tSucc = 360 + tSucc; }

          // get index of the successor
          idxSucc = trunc(tSucc / 5) * width * height + trunc(ySucc) * width + trunc(xSucc);

          // ensure successor is on grid ROW MAJOR^2
          if (xSucc >= 0 && xSucc < width && ySucc >= 0 && ySucc < height && trunc(tSucc / 5) >= 0 &&
              trunc(tSucc / 5) < depth) {

            // ensure successor is not blocked by obstacle  && obstacleBloating(xSucc, ySucc)
            if (grid->data[(int)ySucc * width + (int)xSucc] == 0) {

              // ensure successor is not on closed list or it has the same index
              if (closed[idxSucc] == false || idx == idxSucc) {
                Node3D* nSucc;
                nSucc = new Node3D(xSucc, ySucc, tSucc, nPred->getG(), 0, nullptr);

                // calculate new g value
                nSucc->updateG(*nPred);
                float newG = nSucc->getG();

                // if successor not on open list or found a shorter way to the cell
                if (open[idxSucc] == false || newG < cost[idxSucc]) {

                    // DEBUG if successor is in the same cell
                  // calculate heuristic
                  nSucc->updateH(goal, grid, cost2d);
                  if (idx == idxSucc && nSucc->getH() < nPred->getH()) {
                    std::cout << idx << " entered occupied cell\n";
                    // set predecessor to predecessor of predecessor
                    nSucc->setPred(nPred->getPred());
                  } else {
                    //set predecessor
                    nSucc->setPred(nPred);
                  }

                  // set costs
                  cost[idxSucc] = nSucc->getG();
                  costToGo[idxSucc] = nSucc->getH();
                  // put successor on open list
                  open[idxSucc] = true;
                  O.push(nSucc);
                }

                /*
                   // some new function testing for same fields
                  nSucc->updateH(goal, grid, cost2d);
                  float newH = nPred->getH();
                  else if (newH < costToGo[idxSucc] && nPred->getIdx(width, height) == nSucc->getIdx(width, height)) {
                  // set predecessor
                  nSucc->setPred(nPred->getPred());
                  nSucc->updateG(*nPred);
                  cost[idxSucc] = nSucc->getG();
                  nSucc->updateH(goal, grid, cost2d);
                  costToGo[idxSucc] = nSucc->getH();
                  // put successor on open list
                  open[idxSucc] = true;
                  O.push(nSucc);
                  }
                */
                else { delete nSucc; }
              }
            }
          }
        }
      }
    }
  }

  // return 0
  if (O.empty()) {
    return nullptr;
  }
}
