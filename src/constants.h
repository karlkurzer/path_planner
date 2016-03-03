#ifndef CONSTANTS
#define CONSTANTS

#include <cmath>

namespace constants {
// _________________
// CONFIG FLAGS

// flag -- to turn on the dubins heuristic
const bool dubins = false;

// flag -- to turn on the 2D heuristic
const bool twoD = false;

// flag -- switch to false for velodyne data
static const bool manual = true;

// flag -- switch to true to activate obstacle bloating
static const bool obstacleBloating = false;

// _________________
// GENERAL CONSTANTS

// [m] -- uniformly added padding around the vehicle
static const double bloating = 0;

// [m] -- width of the vehicle
static const double width = 1.75 + bloating;

// [m] -- length of the vehicle
static const double length = 2.65 + bloating;

// [m] -- minimum turning radius of the vehicle
const float r = 5;

// [#] -- number of discretizations in heading
static const int headings = 72;

// [°] -- discretization value of heading == goal condition
static const float deltaHeadingDeg = 360 / headings;

// [c*PI] -- discretization value of heading
static const float deltaHeadingRad = 2 * M_PIl / headings;

// [m] -- cell size in [m]
static const float cellSize = 1;

// _______________
// LOOKUP SPECIFIC

// [m] -- bounding box size length/width
static const int bbSize = std::ceil((sqrt(width * width + length* length) + 2 * bloating) / cellSize) + 4;

// [i] -- relative position from center of the vehicle
struct relPos {
  int x;
  int y;
};

//
struct config {
  int length;
  relPos pos[bbSize];
};

}

#endif // CONSTANTS

