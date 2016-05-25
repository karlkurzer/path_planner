#include "collisiondetection.h"

using namespace HybridAStar;

CollisionDetection::CollisionDetection() {
  this->grid = nullptr;
  Lookup::collisionLookup(collisionLookup);
}

//template<typename T> bool CollisionDetection::isTraversable(const T* node) {
//  /* Depending on the used collision checking mechanism this needs to be adjusted
//     standard: collision checking using the spatial occupancy enumeration
//     other: collision checking using the 2d costmap and the navigation stack
//  */
//  float cost = 0;
//  float x;
//  float y;
//  float t;
//  // assign values to the configuration
//  getConfiguration(node, x, y, t);

//  if (true) {
//    cost = configurationTest(x, y, t) ? 0 : 1;
//  } else {
//    cost = configurationCost(x, y, t);
//  }

//  return cost <= 0;
//}

bool CollisionDetection::configurationTest(float x, float y, float t) {
  int X = (int)x;
  int Y = (int)y;
  int iX = (int)((x - (long)x) * Constants::positionResolution);
  iX = iX > 0 ? iX : 0;
  int iY = (int)((y - (long)y) * Constants::positionResolution);
  iY = iY > 0 ? iY : 0;
  int iT = (int)(t / Constants::deltaHeadingRad);
  int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
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
