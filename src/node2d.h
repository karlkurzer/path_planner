#ifndef NODE2D_H
#define NODE2D_H

#include <cmath>

#include "nav_msgs/OccupancyGrid.h"

class Node2D {
  public:
    // CONSTRUCTOR
    Node2D(int x, int y, float g, float h, Node2D* pred) {
        this->x = x;
        this->y = y;
        this->g = g;
        this->h = h;
        this->pred = pred;
    }
    // GETTER METHODS
    int getX() const { return x; }
    int getY() const { return y; }
    float getG() const { return g; }
    float getH() const { return h; }
    Node2D* getPred() const { return pred; }
    // return total estimated cost for node
    float getC() const { return g + h; }

    // SETTER METHODS
    void setX(const int& x) { this->x = x; }
    void setY(const int& y) { this->y = y; }
    void setG(const float& g) { this->g = g; }
    void setH(const float& h) { this->h = h; }
    void setPred(Node2D* pred) { this->pred = pred; }

    // UPDATE METHODS
    // from predecessor
    void updateG(const Node2D& pred) { g += movementCost(pred); }
    // to goal
    void updateH(const Node2D& goal) { h = movementCost(goal); }
    // euclidean distance
    float movementCost(const Node2D& pred) const { return sqrt((x - pred.x) * (x - pred.x) + (y - pred.y) * (y - pred.y)); }
    // aStar algorithm
    static float aStar(Node2D& start, Node2D& goal, const nav_msgs::OccupancyGrid::ConstPtr& oGrid);

    // CONSTANT VALUES
    // possible directions
    static const int dir;
    // possible movements
    static const int dx[];
    static const int dy[];
  private:
    // x = position (length), y = position (width), g = cost, h = cost to go, pred = pointer to predecessor node
    int x;
    int y;
    float g;
    float h;
    Node2D* pred;
};

#endif // NODE2D_H
