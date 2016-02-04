#include <functional>
#include <queue>
#include <vector>

#include "node3d.h"

// CONSTANT VALUES
// possible directions
const int Node3D::dir = 3;
// possible movements
const float Node3D::dx[] = { 0,    -0.07387, 0.07387};
const float Node3D::dy[] = { 0.92, 0.938607, 0.938607};
const float Node3D::dt[] = { 0,     9,   -9};

//###################################################
//                                      MOVEMENT COST
//###################################################
float Node3D::movementCost(const Node3D& pred) const {
  bool penalty = false;
  float distance, tPenalty = 0;

  if (penalty) {
    // turning penalty
    if (abs(t - pred.getT()) > 0) { tPenalty = 1;}
  }

  // euclidean distance
  distance = sqrt((x - pred.x) * (x - pred.x) + (y - pred.y) * (y - pred.y));
  if (t - pred.getT() == dt[2]) {
    distance = 0.942477796;
  }
  if (t - pred.getT() == dt[1]) {
    distance = 0.942477796;
  }
  if (t - pred.getT() == dt[0]) {
    distance = 0.92;
  }
  return distance + tPenalty;
}

//###################################################
//                                         COST TO GO
//###################################################
float Node3D::costToGo(const Node3D& goal,
                       const nav_msgs::OccupancyGrid::ConstPtr& oGrid,
                       float costGoal[]) const {
  bool dubins = false;
  bool twoD = false;
  float dubinsCost = 0;
  float euclideanCost = 0;

  // if dubins heuristic is activated calculate the shortest path
  // constrained without obstacles
  if (dubins) {
    // start
    double q0[] = { x, y, t / 180 * M_PI };
    // goal
    double q1[] = { goal.x, goal.y, goal.t / 180 * M_PI };
    // minimum turning radius
    float r = 1.5;
    DubinsPath path;
    dubins_init(q0, q1, r, &path);
    dubinsCost = dubins_path_length(&path);
  }

  // if twoD heuristic is activated determine shortest path
  // unconstrained with obstacles
  if (twoD && costGoal[(int)y * oGrid->info.width + (int)x] == 0) {
    // create a 2d start node
    Node2D start2d(x, y, 0, 0, nullptr);
    // create a 2d goal node
    Node2D goal2d(goal.x, goal.y, 0, 0, nullptr);
    // run 2d astar and return the cost of the cheapest path for that node
    costGoal[(int)y * oGrid->info.width + (int)x] = Node2D::aStar(start2d, goal2d, oGrid);
  }

  // else calculate the euclidean distance
  euclideanCost = sqrt((x - goal.x) * (x - goal.x) + (y - goal.y) * (y - goal.y));
  // return the maximum of the heuristics, making the heuristic admissable
  return std::max(euclideanCost, std::max(dubinsCost, costGoal[(int)y * oGrid->info.width + (int)x]));
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
  return lhs.getX() == rhs.getX() && lhs.getY() == rhs.getY() &&
         std::abs(std::abs(lhs.getT()) - std::abs(rhs.getT())) <= 5;
}

//###################################################
//                                 				3D A*
//###################################################
Node3D* Node3D::aStar(Node3D& start, const Node3D& goal,
                      const nav_msgs::OccupancyGrid::ConstPtr& oGrid, int width, int height, int depth, int length,
                      bool* open, bool* closed, float* cost, float* costToGo, float* costGoal) {

  // PREDECESSOR AND SUCCESSOR POSITION
  float x, y, t, xSucc, ySucc, tSucc;
  int idx = 0;
  int idxSucc = 0;

  // OPEN LIST
  std::priority_queue<Node3D*, std::vector<Node3D*>, CompareNodes> O;
  // update g value
  start.updateG(start);
  // update h value
  start.updateH(goal, oGrid, costGoal);
  // push on priority queue
  O.push(&start);
  // add node to open list with total estimated cost
  cost[start.getIdx(width, height)] = start.getC();

  // continue until O empty
  while (!O.empty()) {
    // create new node pointer
    Node3D* nPred;
    // pop node with lowest cost from priority queue
    nPred = O.top();
    x = nPred->getX();
    y = nPred->getY();
    t = nPred->getT();
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
      if (nPred->getIdx(width, height) == goal.getIdx(width, height)) {
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
            if (oGrid->data[(int)ySucc * width + (int)xSucc] == 0) {

              // ensure successor is not on closed list
              if (closed[idxSucc] == false) {
                Node3D* nSucc;
                nSucc = new Node3D(xSucc, ySucc, tSucc, nPred->getG(), 0, nullptr);

                // calculate new g value
                float newG = nPred->getG() + nSucc->movementCost(*nPred);

                // if successor not on open list or g value lower than before put it on open list
                if (open[idxSucc] == false || newG < cost[idxSucc]) {
                  // set predecessor
                  nSucc->setPred(nPred);
                  nSucc->updateG(*nPred);
                  cost[idxSucc] = nSucc->getG();
                  nSucc->updateH(goal, oGrid, costGoal);
                  costToGo[idxSucc] = nSucc->getH();
                  // put successor on open list
                  open[idxSucc] = true;
                  O.push(nSucc);
                } else { delete nSucc; }
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
