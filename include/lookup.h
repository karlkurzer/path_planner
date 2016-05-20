#ifndef COLLISIONLOOKUP
#define COLLISIONLOOKUP

#include "dubins.h"
#include "constants.h"

namespace HybridAStar {
namespace Lookup {

//###################################################
//                                      DUBINS LOOKUP
//###################################################
inline void dubinsLookup(float* lookup) {
  bool DEBUG = false;
  std::cout << "I am building the Dubin's lookup table...";

  DubinsPath path;

  int width = Constants::dubinsWidth / Constants::cellSize;

  //  // increase the width by one to make it square
  //  if (width % 2 != 0) {
  //    width++;
  //  }

  const int headings = Constants::headings;

  // start and goal vector
  double start[3];
  double goal[] = {0, 0, 0};

  // iterate over the X index of a grid cell
  for (int X = 0; X < width; ++X) {
    start[0] = X;

    // iterate over the Y index of a grid cell
    for (int Y = 0; Y < width; ++Y) {
      start[1] = Y;

      // iterate over the start headings
      for (int h0 = 0; h0 < headings; ++h0) {
        start[2] = Constants::deltaHeadingRad * h0;

        // iterate over the goal headings
        for (int h1 = 0; h1 < headings; ++h1) {
          goal[2] = Constants::deltaHeadingRad * h1;

          // calculate the actual cost
          dubins_init(start, goal, Constants::r, &path);
          lookup[X * headings * headings * width + Y * headings * headings + h0 * headings + h1] = dubins_path_length(&path);

          if (DEBUG && lookup[X * headings * headings * width + Y * headings * headings + h0 * headings + h1] < sqrt(X * X + Y * Y) * 1.000001) {
            std::cout << X << " | " << Y << " | "
                      << Constants::deltaHeadingDeg* h0 << " | "
                      << Constants::deltaHeadingDeg* h1 << " length: "
                      << lookup[X * headings * headings * width + Y * headings * headings + h0 * headings + h1] << "\n";

          }
        }
      }
    }
  }

  std::cout << " done!" << std::endl;
}

//###################################################
//                                   COLLISION LOOKUP
//###################################################

// _____________
// SIGN FUNCTION
inline int sign(double x) {
  if (x >= 0) { return 1; }
  else { return -1; }
}

// _________________________
// COLLISION LOOKUP CREATION
inline void collisionLookup(Constants::config* lookup) {
  bool DEBUG = false;
  std::cout << "I am building the collision lookup table...";
  // cell size
  const float cSize = Constants::cellSize;
  // bounding box size length/width
  const int size = Constants::bbSize;

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
  double theta;

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
  int hcross1 = 0;
  int hcross2 = 0;

  // _____________________________
  // VARIABLES FOR LOOKUP CREATION
  int count = 0;
  const int positionResolution = Constants::positionResolution;
  const int positions = Constants::positions;
  point points[positions];

  // generate all discrete positions within one cell
  for (int i = 0; i < positionResolution; ++i) {
    for (int j = 0; j < positionResolution; ++j) {
      points[positionResolution * i + j].x = 1.f / positionResolution * j;
      points[positionResolution * i + j].y = 1.f / positionResolution * i;
    }
  }


  for (int q = 0; q < positions; ++q) {
    // set the starting angle to zero;
    theta = 0;

    // set points of rectangle
    c.x = (double)size / 2 + points[q].x;
    c.y = (double)size / 2 + points[q].y;

    p[0].x = c.x - Constants::length / 2 / cSize;
    p[0].y = c.y - Constants::width / 2 / cSize;

    p[1].x = c.x - Constants::length / 2 / cSize;
    p[1].y = c.y + Constants::width / 2 / cSize;

    p[2].x = c.x + Constants::length / 2 / cSize;
    p[2].y = c.y + Constants::width / 2 / cSize;

    p[3].x = c.x + Constants::length / 2 / cSize;
    p[3].y = c.y - Constants::width / 2 / cSize;

    for (int o = 0; o < Constants::headings; ++o) {
      if (DEBUG) { std::cout << "\ndegrees: " << theta * 180.f / M_PI << std::endl; }

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
      }

      // create the next angle
      theta += Constants::deltaHeadingRad;

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
        X = (int)start.x;
        Y = (int)start.y;
        //      std::cout << "StartCell: " << X << "," << Y << std::endl;
        cSpace[Y * size + X] = true;
        t.x = end.x - start.x;
        t.y = end.y - start.y;
        stepX = sign(t.x);
        stepY = sign(t.y);

        // width and height normalized by t
        if (t.x != 0) {
          tDeltaX = 1.f / std::abs(t.x);
        } else {
          tDeltaX = 1000;
        }

        if (t.y != 0) {
          tDeltaY = 1.f / std::abs(t.y);
        } else {
          tDeltaY = 1000;
        }

        // set maximum traversal values
        if (stepX > 0) {
          tMaxX = tDeltaX * (1 - (start.x - (long)start.x));
        } else {
          tMaxX = tDeltaX * (start.x - (long)start.x);
        }

        if (stepY > 0) {
          tMaxY = tDeltaY * (1 - (start.y - (long)start.y));
        } else {
          tMaxY = tDeltaY * (start.y - (long)start.y);
        }

        while ((int)end.x != X || (int)end.y != Y) {
          // only increment x if the t length is smaller and the result will be closer to the goal
          if (tMaxX < tMaxY && std::abs(X + stepX - (int)end.x) < std::abs(X - (int)end.x)) {
            tMaxX = tMaxX + tDeltaX;
            X = X + stepX;
            cSpace[Y * size + X] = true;
            // only increment y if the t length is smaller and the result will be closer to the goal
          } else if (tMaxY < tMaxX && std::abs(Y + stepY - (int)end.y) < std::abs(Y - (int)end.y)) {
            tMaxY = tMaxY + tDeltaY;
            Y = Y + stepY;
            cSpace[Y * size + X] = true;
          } else if (2 >= std::abs(X - (int)end.x) + std::abs(Y - (int)end.y)) {
            if (std::abs(X - (int)end.x) > std::abs(Y - (int)end.y)) {
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
            lookup[q * Constants::headings + o].pos[count].x = j - (int)c.x;
            lookup[q * Constants::headings + o].pos[count].y = i - (int)c.y;
            // add one for the length of the current list
            count++;
          }
        }
      }

      lookup[q * Constants::headings + o].length = count;

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
        std::cout << "\n\nthe center of " << q* Constants::headings + o << " is at " << c.x << " | " << c.y << std::endl;

        for (int i = 0; i < lookup[q * Constants::headings + o].length; ++i) {
          std::cout << "[" << i << "]\t" << lookup[q * Constants::headings + o].pos[i].x << " | " << lookup[q * Constants::headings + o].pos[i].y << std::endl;
        }
      }
    }
  }

  std::cout << " done!" << std::endl;
}

}
}
#endif // LOOKUP

