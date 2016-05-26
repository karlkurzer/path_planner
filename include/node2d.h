#ifndef NODE2D_H
#define NODE2D_H

#include <cmath>

#include "constants.h"
namespace HybridAStar {

/*!
   \brief A two dimensional node class used for the holonomic with obstacles heuristic.

   Each node has a unique discrete position (x,y).
*/
class Node2D {
 public:
  /// The default constructor for 2D array initialization.
  Node2D(): Node2D(0, 0, 0, 0, nullptr) {}
  /// Constructor for a node with the given arguments
  Node2D(int x, int y, float g, float h, Node2D* pred) {
    this->x = x;
    this->y = y;
    this->g = g;
    this->h = h;
    this->pred = pred;
    this->o = false;
    this->c = false;
    this->d = false;
    this->idx = -1;
  }
  // GETTER METHODS
  /// get the x position
  int getX() const { return x; }
  /// get the y position
  int getY() const { return y; }
  /// get the cost-so-far (real value)
  float getG() const { return g; }
  /// get the cost-to-come (heuristic value)
  float getH() const { return h; }
  /// get the total estimated cost
  float getC() const { return g + h; }
  /// get the index of the node in the 2D array
  int getIdx() const { return idx; }
  /// determine whether the node is open
  bool  isOpen() const { return o; }
  /// determine whether the node is closed
  bool  isClosed() const { return c; }
  /// determine whether the node is discovered
  bool  isDiscovered() const { return d; }
  /// get a pointer to the predecessor
  Node2D* getPred() const { return pred; }

  // SETTER METHODS
  /// set the x position
  void setX(const int& x) { this->x = x; }
  /// set the y position
  void setY(const int& y) { this->y = y; }
  /// set the cost-so-far (real value)
  void setG(const float& g) { this->g = g; }
  /// set the cost-to-come (heuristic value)
  void setH(const float& h) { this->h = h; }
  /// set and get the index of the node in the 2D array
  int setIdx(int width) { this->idx = y * width + x; return idx;}
  /// open the node
  void open() { o = true; c = false; }
  /// close the node
  void close() { c = true; o = false; }
  /// set the node neither open nor closed
  void reset() { c = false; o = false; }
  /// discover the node
  void discover() { d = true; }
  /// set a pointer to the predecessor of the node
  void setPred(Node2D* pred) { this->pred = pred; }

  // UPDATE METHODS
  /// Updates the cost-so-far for the node x' coming from its predecessor. It also discovers the node.
  void updateG() { g += movementCost(*pred); d = true; }
  /// Updates the cost-to-go for the node x' to the goal node.
  void updateH(const Node2D& goal) { h = movementCost(goal); }
  /// The heuristic as well as the cost measure.
  float movementCost(const Node2D& pred) const { return sqrt((x - pred.x) * (x - pred.x) + (y - pred.y) * (y - pred.y)); }

  // CUSTOM OPERATORS
  /// Custom operator to compare nodes. Nodes are equal if their x and y position is the same.
  bool operator == (const Node2D& rhs) const;

  // GRID CHECKING
  /// Validity check to test, whether the node is in the 2D array.
  bool isOnGrid(const int width, const int height) const;

  // SUCCESSOR CREATION
  /// Creates a successor on a eight-connected grid.
  Node2D* createSuccessor(const int i);

  // CONSTANT VALUES
  /// Number of possible directions
  static const int dir;
  /// Possible movements in the x direction
  static const int dx[];
  /// Possible movements in the y direction
  static const int dy[];

 private:
  /// the x position
  int x;
  /// the y position
  int y;
  /// the cost-so-far
  float g;
  /// the cost-to-go
  float h;
  /// the index of the node in the 2D array
  int idx;
  /// the open value
  bool o;
  /// the closed value
  bool c;
  /// the discovered value
  bool d;
  /// the predecessor pointer
  Node2D* pred;
};
}
#endif // NODE2D_H
