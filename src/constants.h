#ifndef CONSTANTS
#define CONSTANTS

namespace constants {
// _________________
// GENERAL CONSTANTS

// [m] -- width of the vehicle
static const double width = 1.75;

// [m] -- length of the vehicle
static const double length = 2.65;

// [m] -- uniformly added padding around the vehicle
static const double bloating = 0;

// [#] -- number of discretizations in heading
static const int headings = 72;

// [°] -- discretization value of heading
static const float deltaHeadingDeg = 360/orientations;

// [c*PI] -- discretization value of heading
static const float deltaHeadingRad = 3.14159265359/orientations;

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

