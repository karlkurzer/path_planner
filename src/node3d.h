#ifndef NODE3D_H
#define NODE3D_H

#include <cmath>

#include "nav_msgs/OccupancyGrid.h"

#include "dubins.h"
#include "node2d.h"

struct cfg;

class Node3D {
  public:
    // CONSTRUCTOR
    Node3D(int x, int y, float t, float g, float h, Node3D* pred) {
        this->x = x;
        this->y = y;
        this->t = t;
        this->g = g;
        this->h = h;
        this->pred = pred;
    }
    // GETTER METHODS
    int getX() const { return x; }
    int getY() const { return y; }
    float getT() const { return t; }
    float getG() const { return g; }
    float getH() const { return h; }
    Node3D* getPred() const { return pred; }
    // returns total estimated cost for node
    float getC() const { return g + h; }

    // SETTER METHODS
    void setX(const int& x) { this->x = x; }
    void setY(const int& y) { this->y = y; }
    void setT(const float& t) { this->t = t; }
    void setG(const float& g) { this->g = g; }
    void setH(const float& h) { this->h = h; }
    void setPred(Node3D* pred) { this->pred = pred; }

    // UPDATE METHODS
    // from start
    void updateG(const Node3D& pred) { g += movementCost(pred); }
    // to goal
    void updateH(const Node3D& goal, const nav_msgs::OccupancyGrid::ConstPtr& grid) { h = costToGo(goal, grid); }
    // COST CALCULATION
    // cost for movement, g
    float movementCost(const Node3D& pred) const {
        bool penalty = false;
        float distance, tPenalty = 0;

        if (penalty) {
            //heading penalty
            if (abs(t - pred.getT()) > 180) { tPenalty = (360 - abs(t - pred.getT())) / 45; }
            else { tPenalty = abs(t - pred.getT()) / 45; }
        }

        // euclidean distance
        distance = sqrt((x - pred.x) * (x - pred.x) + (y - pred.y) * (y - pred.y));
        return distance + tPenalty;
    }
    // cost to go, dubins path / 2D A*
    float costToGo(const Node3D& goal, const nav_msgs::OccupancyGrid::ConstPtr& oGrid) const {
        bool dubins = false;
        bool twoD = false;
        float cost = 0, dubinsLength = 0, euclidean = 0;
        int newT = 0, newGoalT = 0;

        if (dubins) {
            // theta conversion
            newT = (int)((360 - t) + 180) % 360;
            newGoalT = (int)((360 - goal.t) + 180) % 360;
            //start
            double q0[] = { x, y, newT / 180 * M_PI };
            // goal
            double q1[] = { goal.x, goal.y, newGoalT / 180 * M_PI };
            // turning radius
            float r = 1;
            DubinsPath path;
            dubins_init(q0, q1, r, &path);
            dubinsLength = dubins_path_length(&path);
        }

        if (twoD && costGoal[y * oGrid->info.width + x] == 0) {
            Node2D start2d(x, y, 0, 0, nullptr);
            Node2D goal2d(goal.x, goal.y, 0, 0, nullptr);
            costGoal[y * oGrid->info.width + x] = Node2D::aStar(start2d, goal2d, oGrid);
        }

        euclidean = sqrt((x - goal.x) * (x - goal.x) + (y - goal.y) * (y - goal.y));
        return std::max(euclidean, std::max(dubinsLength, costGoal[y * oGrid->info.width + x]));
    }
    //  aStar algorithm
    static float aStar(Node3D& start, const Node3D& goal, const nav_msgs::OccupancyGrid::ConstPtr& oGrid);
    // CONSTANT VALUES
    // possible directions
    static const int dir;
    // possible movements
    static const int dx[];
    static const int dy[];
    static const int dt[];
    static float costGoal[];
  private:
    // x = position (length), y = position (width), t = heading, g = cost, h = cost to go, pred = pointer to predecessor node
    int x;
    int y;
    float t;
    float g;
    float h;
    Node3D* pred;
};

////###################################################
////                                  CONST DECLARATION
////###################################################
///*
//    HEADING => 0 - 359 degrees, 0 being north pointing towards negative X
//    X-COORDINATE => designating the length of the grid/world
//    Y-COORDINATE => designating the width of the grid/world
//*/

//// PATH TRACE
//bool path[length][width];
//int pathClosed[length][width];
//// ALGORITHM LISTS
//// 2D
//bool listOpen2D[length][width];
//bool listClosed2D[length][width];
//float listCost2D[length][width];
//float listCostToGo2D[length][width];
//float listCostGoal2D[length][width];

//// USER CHOICE
//bool penalty = true;
//bool dubins = true;
//bool twoD = true;

#endif // NODE3D_H
