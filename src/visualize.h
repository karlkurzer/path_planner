#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <ros/ros.h>

#include "node3d.h"


class Visualize {
 public:
  // ___________
  // CONSTRUCTOR
  Visualize();

  void publish3dNode(Node3D node) const;

 private:

};

#endif // VISUALIZE_H
