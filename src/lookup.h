#ifndef COLLISIONLOOKUP
#define COLLISIONLOOKUP

#include "constants.h"

namespace lookup {
//###################################################
//                                    COLLISIONLOOKUP
//###################################################

// _____________
// SIGN FUNCTION
int sign(double x) {
  if (x >= 0) { return 1; }

  if (x < 0) { return -1; }
}

// _______________
// LOOKUP CREATION
void collisionLookup(constants::config* lookup) {
  bool DEBUG = true;
  // cell size
  const float cSize = constants::cellSize;
  // bounding box size length/width
  const int size = constants::bbSize;

  struct point {
    double x;
    double y;
  };

  // ______________________
  // VARIABLES FOR ROTATION
  //center of the rectangle
  point c;
  point temp;
  // points of the rectangle
  point p[4];
  point nP[4];

  // turning angle
  double theta = 0;

  // set points of rectangle
  c.x = (double)size / 2;
  c.y = (double) size / 2;

  p[0].x = c.x - constants::width / 2 / cSize;
  p[0].y = c.y - constants::length / 2 / cSize;

  p[1].x = c.x - constants::width / 2 / cSize;
  p[1].y = c.y + constants::length / 2 / cSize;

  p[2].x = c.x + constants::width / 2 / cSize;
  p[2].y = c.y + constants::length / 2 / cSize;

  p[3].x = c.x + constants::width / 2 / cSize;
  p[3].y = c.y - constants::length / 2 / cSize;


  // ____________________________
  // VARIABLES FOR GRID TRAVERSAL
  // vector for grid traversal
  point t;
  point start;
  point end;
  // cell index
  int X;
  int Y;
  // t value for crossing vertical and horizontal boundary
  double tMaxX;
  double tMaxY;
  // t value for width/heigth of cell
  double tDeltaX;
  double tDeltaY;
  // positive or negative step direction
  int stepX;
  int stepY;
  // grid
  bool cSpace[size * size];
  bool inside = false;
  int hcross1;
  int hcross2;

  // _____________________________
  // VARIABLES FOR LOOKUP CREATION
  int count = 0;

  for (int o = 0; o < constants::headings; ++o) {
    if (DEBUG) { std::cout << "\ndegrees: " << theta * 180 / M_PI << std::endl; }

    // initialize cSpace
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        cSpace[i * size + j] = false;
      }
    }

    // shape rotation
    for (int j = 0; j < 4; ++j) {
      // translate point to origin
      temp.x = p[j].x - c.x;
      temp.y = p[j].y - c.y;

      // rotate and shift back
      nP[j].x = temp.x * cos(theta) - temp.y * sin(theta) + c.x;
      nP[j].y = temp.x * sin(theta) + temp.y * cos(theta) + c.y;
      //      std::cout << j << "--> pX: " << nP[j].x << " pY: " << nP[j].y << std::endl;
    }

    // create the next angle
    theta += constants::deltaHeadingRad;

    // cell traversal clockwise
    for (int k = 0; k < 4; ++k) {
      // create the vectors clockwise
      if (k < 3) {
        start = nP[k];
        end = nP[k + 1];
      } else {
        start = nP[k];
        end = nP[0];
      }

      //set indexes
      X = trunc(start.x);
      Y = trunc(start.y);
      //      std::cout << "StartCell: " << X << "," << Y << std::endl;
      cSpace[Y * size + X] = true;
      t.x = end.x - start.x;
      t.y = end.y - start.y;
      stepX = sign(t.x);
      stepY = sign(t.y);

      // width and height normalized by t
      if (t.x != 0) {
        tDeltaX = cSize / std::abs(t.x);
      } else {
        tDeltaX = 1000;
      }

      if (t.y != 0) {
        tDeltaY = cSize / std::abs(t.y);
      } else {
        tDeltaY = 1000;
      }

      // set maximum traversal values
      if (stepX > 0) {
        tMaxX = tDeltaX * (1 - (start.x / cSize - (long)(start.x / cSize)));
      } else {
        tMaxX = tDeltaX * (1 - (1 - (start.x / cSize - (long)(start.x / cSize))));
      }

      if (stepY > 0) {
        tMaxY = tDeltaY * (1 - (start.y / cSize - (long)(start.y / cSize)));
      } else {
        tMaxY = tDeltaY * (1 - (1 - (start.y / cSize - (long)(start.y / cSize))));
      }

      while (trunc(end.x) != X || trunc(end.y) != Y) {
        // only increment x if the t length is smaller and the result will be closer to the goal
        if (tMaxX < tMaxY && std::abs(X + stepX - trunc(end.x)) < std::abs(X - trunc(end.x))) {
          tMaxX = tMaxX + tDeltaX;
          X = X + stepX;
          cSpace[Y * size + X] = true;
          //          std::cout << "Cell: " << X << "," << Y << std::endl;
          // only increment y if the t length is smaller and the result will be closer to the goal
        } else if (tMaxY < tMaxX && std::abs(Y + stepY - trunc(end.y)) < std::abs(Y - trunc(end.y))) {
          tMaxY = tMaxY + tDeltaY;
          Y = Y + stepY;
          cSpace[Y * size + X] = true;
          //          std::cout << "Cell: " << X << "," << Y << std::endl;
        } else if (2 >= std::abs(X - trunc(end.x)) + std::abs(Y - trunc(end.y))) {
          if (std::abs(X - trunc(end.x)) > std::abs(Y - trunc(end.y))) {
            X = X + stepX;
            cSpace[Y * size + X] = true;
          } else {
            Y = Y + stepY;
            cSpace[Y * size + X] = true;
          }
        } else {
          // this SHOULD NOT happen
          std::cout << "\n--->tie occured, please check for error in script\n";
          break;
        }
      }
    }

    // FILL THE SHAPE
    for (int i = 0; i < size; ++i) {
      // set inside to false
      inside = false;

      for (int j = 0; j < size; ++j) {

        // determine horizontal crossings
        for (int k = 0; k < size; ++k) {
          if (cSpace[i * size + k] && !inside) {
            hcross1 = k;
            inside = true;
          }

          if (cSpace[i * size + k] && inside) {
            hcross2 = k;
          }
        }

        // if inside fill
        if (j > hcross1 && j < hcross2 && inside) {
          cSpace[i * size + j] = true;
        }
      }
    }

    // GENERATE THE ACTUAL LOOKUP
    count = 0;

    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        if (cSpace[i * size + j]) {
          // compute the relative position of the car cells
          lookup[o].pos[count].x = j + 1 - trunc(c.x);
          lookup[o].pos[count].y = i + 1 - trunc(c.y);
          // add one for the length of the current list
          count++;
        }
      }
    }

    lookup[o].length = count;

    if (DEBUG) {
      //DEBUG
      for (int i = 0; i < size; ++i) {
        std::cout << "\n";

        for (int j = 0; j < size; ++j) {
          if (cSpace[i * size + j]) {
            std::cout << "#";
          } else {
            std::cout << ".";
          }
        }
      }

      //TESTING
      std::cout << "\n\nthe center is at " << c.x << " | " << c.y << std::endl;

      for (int i = 0; i < lookup[o].length; ++i) {
        std::cout << "[" << i << "]\t" << lookup[o].pos[i].x << " | " << lookup[o].pos[i].y << std::endl;
      }
    }
  }
}

}

#endif // LOOKUP

