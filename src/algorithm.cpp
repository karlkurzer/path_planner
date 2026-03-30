#include "algorithm.h"

#include <boost/heap/binomial_heap.hpp>

using namespace HybridAStar;

float aStar(Node2D& start, Node2D& goal, Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization, rclcpp::Node::SharedPtr n);
void updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization, rclcpp::Node::SharedPtr n);
Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace);

struct CompareNodes {
  bool operator()(const Node3D* lhs, const Node3D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
  bool operator()(const Node2D* lhs, const Node2D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
};

Node3D* Algorithm::hybridAStar(Node3D& start,
                               const Node3D& goal,
                               Node3D* nodes3D,
                               Node2D* nodes2D,
                               int width,
                               int height,
                               CollisionDetection& configurationSpace,
                               float* dubinsLookup,
                               Visualize& visualization,
                               rclcpp::Node::SharedPtr n) {

  int iPred, iSucc;
  float newG;
  int dir = Constants::reverse ? 6 : 3;
  int iterations = 0;

  typedef boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>> priorityQueue;
  priorityQueue O;

  updateH(start, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization, n);
  start.open();
  O.push(&start);
  iPred = start.setIdx(width, height);
  nodes3D[iPred] = start;

  Node3D* nPred;
  Node3D* nSucc;

  while (!O.empty()) {
    nPred = O.top();
    iPred = nPred->setIdx(width, height);
    iterations++;

    if (Constants::visualization) {
      visualization.publishNode3DPoses(*nPred);
      visualization.publishNode3DPose(*nPred);
    }

    if (nodes3D[iPred].isClosed()) {
      O.pop();
      continue;
    }
    else if (nodes3D[iPred].isOpen()) {
      nodes3D[iPred].close();
      O.pop();

      if (*nPred == goal || iterations > Constants::iterations) {
        return nPred;
      }
      else {
        if (Constants::dubinsShot && nPred->isInRange(goal) && nPred->getPrim() < 3) {
          nSucc = dubinsShot(*nPred, goal, configurationSpace);
          if (nSucc != nullptr && *nSucc == goal) {
            return nSucc;
          }
        }

        for (int i = 0; i < dir; i++) {
          nSucc = nPred->createSuccessor(i);
          iSucc = nSucc->setIdx(width, height);

          if (nSucc->isOnGrid(width, height) && configurationSpace.isTraversable(nSucc)) {
            if (!nodes3D[iSucc].isClosed() || iPred == iSucc) {
              nSucc->updateG();
              newG = nSucc->getG();

              if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc) {
                updateH(*nSucc, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization, n);

                if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) {
                  delete nSucc;
                  continue;
                }
                else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker) {
                  nSucc->setPred(nPred->getPred());
                }

                nSucc->open();
                nodes3D[iSucc] = *nSucc;
                O.push(&nodes3D[iSucc]);
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
  return nullptr;
}

float aStar(Node2D& start,
            Node2D& goal,
            Node2D* nodes2D,
            int width,
            int height,
            CollisionDetection& configurationSpace,
            Visualize& visualization,
            rclcpp::Node::SharedPtr n) {

  int iPred, iSucc;
  float newG;

  for (int i = 0; i < width * height; ++i) {
    nodes2D[i].reset();
  }

  boost::heap::binomial_heap<Node2D*, boost::heap::compare<CompareNodes>> O;
  start.updateH(goal);
  start.open();
  O.push(&start);
  iPred = start.setIdx(width);
  nodes2D[iPred] = start;

  Node2D* nPred;
  Node2D* nSucc;

  while (!O.empty()) {
    nPred = O.top();
    iPred = nPred->setIdx(width);

    if (nodes2D[iPred].isClosed()) {
      O.pop();
      continue;
    }
    else if (nodes2D[iPred].isOpen()) {
      nodes2D[iPred].close();
      nodes2D[iPred].discover();

      if (Constants::visualization2D) {
        visualization.publishNode2DPoses(*nPred);
        visualization.publishNode2DPose(*nPred);
      }

      O.pop();

      if (*nPred == goal) {
        return nPred->getG();
      }
      else {
        for (int i = 0; i < Node2D::dir; i++) {
          nSucc = nPred->createSuccessor(i);
          iSucc = nSucc->setIdx(width);

          if (nSucc->isOnGrid(width, height) &&  configurationSpace.isTraversable(nSucc) && !nodes2D[iSucc].isClosed()) {
            nSucc->updateG();
            newG = nSucc->getG();

            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG()) {
              nSucc->updateH(goal);
              nSucc->open();
              nodes2D[iSucc] = *nSucc;
              O.push(&nodes2D[iSucc]);
              delete nSucc;
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }
  return 1000;
}

void updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization, rclcpp::Node::SharedPtr n) {
  float dubinsCost = 0;
  float reedsSheppCost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;

  if (Constants::dubins) {
    ompl::base::DubinsStateSpace dubinsPath(Constants::r);
    State* dbStart = (State*)dubinsPath.allocState();
    State* dbEnd = (State*)dubinsPath.allocState();
    dbStart->setXY(start.getX(), start.getY());
    dbStart->setYaw(start.getT());
    dbEnd->setXY(goal.getX(), goal.getY());
    dbEnd->setYaw(goal.getT());
    dubinsCost = dubinsPath.distance(dbStart, dbEnd);
    dubinsPath.freeState(dbStart);
    dubinsPath.freeState(dbEnd);
  }

  if (Constants::reverse && !Constants::dubins) {
    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    State* rsStart = (State*)reedsSheppPath.allocState();
    State* rsEnd = (State*)reedsSheppPath.allocState();
    rsStart->setXY(start.getX(), start.getY());
    rsStart->setYaw(start.getT());
    rsEnd->setXY(goal.getX(), goal.getY());
    rsEnd->setYaw(goal.getT());
    reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
    reedsSheppPath.freeState(rsStart);
    reedsSheppPath.freeState(rsEnd);
  }

  if (Constants::twoD && !nodes2D[(int)start.getY() * width + (int)start.getX()].isDiscovered()) {
    Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
    Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);
    nodes2D[(int)start.getY() * width + (int)start.getX()].setG(aStar(goal2d, start2d, nodes2D, width, height, configurationSpace, visualization, n));
  }

  if (Constants::twoD) {
    twoDoffset = sqrt(((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) * ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) +
                      ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())) * ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())));
    twoDCost = nodes2D[(int)start.getY() * width + (int)start.getX()].getG() - twoDoffset;
  }

  start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));
}

Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace) {
  double q0[] = { start.getX(), start.getY(), start.getT() };
  double q1[] = { goal.getX(), goal.getY(), goal.getT() };
  DubinsPath path;
  dubins_init(q0, q1, Constants::r, &path);

  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);

  Node3D* dubinsNodes = new Node3D [(int)(length / Constants::dubinsStepSize) + 1];

  x += Constants::dubinsStepSize;
  while (x <  length) {
    double q[3];
    dubins_path_sample(&path, x, q);
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(Helper::normalizeHeadingRad(q[2]));

    if (configurationSpace.isTraversable(&dubinsNodes[i])) {
      if (i > 0) {
        dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i].setPred(&start);
      }

      x += Constants::dubinsStepSize;
      i++;
    } else {
      delete [] dubinsNodes;
      return nullptr;
    }
  }
  return &dubinsNodes[i - 1];
}
