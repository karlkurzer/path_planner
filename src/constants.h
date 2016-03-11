#ifndef CONSTANTS
#define CONSTANTS

////###################################################
////                                               INFO
////###################################################
//    HEADING => 0 - 359 degrees, 0 being north pointing towards positive Y
//    X-COORDINATE => designating the width of the grid
//    Y-COORDINATE => designating the height of the grid

#include <cmath>

namespace constants {
// _________________
// CONFIG FLAGS

// flag -- switch to true for cout info
static const bool coutDEBUG = true;

// flag -- switch to false for velodyne data
static const bool manual = true;

// flag -- to turn on the dubins shot
static const bool dubinsShot = false;

// flag -- to turn on the dubins heuristic
static const bool dubins = true;

// flag -- to turn on the dubins lookup
static const bool dubinsLookup = false * dubins;

// flag -- to turn on the 2D heuristic
static const bool twoD = true;


// flag -- switch to true to activate obstacle bloating
static const bool obstacleBloating = false;

// _________________
// GENERAL CONSTANTS

// [m] -- uniformly added padding around the vehicle
static const double bloating = 0;

// [m] -- width of the vehicle
static const double width = 1.75 + 2 * bloating;

// [m] -- length of the vehicle
static const double length = 2.65 + 2 * bloating;

// [m] -- minimum turning radius of the vehicle
static const float r = 6;

// [#] -- number of discretizations in heading
static const int headings = 72;

// [°] -- discretization value of heading == goal condition
static const float deltaHeadingDeg = 360 / (float)headings;

// [c*PI] -- discretization value of heading
static const float deltaHeadingRad = 2 * M_PI / (float)headings;

// [m] -- cell size in [m]
static const float cellSize = 1;

// ___________________
// HEURISTIC CONSTANTS

// [#] -- factor to ensure admissibility
static const float factor2D = sqrt(5) / sqrt(2) + 1;

// [#] -- penalty for turning
static const float penaltyTurning = 1.05;

// [m] -- dubins shot step size
static const float dubinsStepSize = 1;

// ______________________
// DUBINS LOOKUP SPECIFIC

// [m] -- width of the dubinsArea / 2
static const int dubinsWidth = 15;
// [m²] -- area of the dubinsArea
static const int dubinsArea = dubinsWidth * dubinsWidth;


// _________________________
// COLLISION LOOKUP SPECIFIC

// [m] -- bounding box size length/width
static const int bbSize = std::ceil(sqrt(width * width + length* length) + 4 / cellSize);

// [#] -- number of discrete per cell length
static const int positionResolution = 10;
// [#] -- number of discrete points in one square
static const int positions = positionResolution * positionResolution;

// [i] -- relative position from center of the vehicle
struct relPos {
  int x;
  int y;
};

//
struct config {
  int length;
  relPos pos[64];
};

}

#endif // CONSTANTS

