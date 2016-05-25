#ifndef CONSTANTS
#define CONSTANTS
/*!
   \file constants.h
   \brief This is a collection of constants that are used throughout the project.
   \todo All constants need to be checked and documented
*/

////###################################################
////                                               INFO
////###################################################
//    HEADING => 0 - 359 degrees, 0 being north pointing towards positive Y
//    X-COORDINATE => designating the width of the grid
//    Y-COORDINATE => designating the height of the grid

#include <cmath>

namespace HybridAStar {
/*!
    \brief The namespace that wraps the constants
    \namespace constants
*/
namespace Constants {
// _________________
// CONFIG FLAGS

/*! \brief A debugging variable for additional output via `std::cout` */
static const bool coutDEBUG = 0;

/*! \brief A variable for the mode, manual for static map or dynamic for dynamic map. */
static const bool manual = 1;

// flag -- switch to true for live visualization
static const bool visualization = 0 * manual;

// flag -- switch to true for live visualization
static const bool visualization2D = 0 * manual;

// flag -- to turn on reversin of the vehicle
static const bool reverse = true;

// flag -- to turn on the dubins shot
static const bool dubinsShot = true;

// flag -- to turn on the dubins heuristic
static const bool dubins = false;

// flag -- to turn on the dubins lookup
static const bool dubinsLookup = false * dubins;

// flag -- to turn on the 2D heuristic
static const bool twoD = true;

// flag -- switch to true to activate obstacle bloating
static const bool obstacleBloating = false;

// _________________
// GENERAL CONSTANTS

// [#] -- maximum search depth
static const int iterations = 10000;

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

// [c*PI] -- goal condition
static const float deltaHeadingNegRad = 2 * M_PI - deltaHeadingRad;

// [m] -- cell size
static const float cellSize = 1;

// [m] -- tie breaker to break ties between nodes in the same cell
static const float tieBreaker = 0.01;

// ___________________
// HEURISTIC CONSTANTS

// [#] -- factor to ensure admissibility
static const float factor2D = sqrt(5) / sqrt(2) + 1;

// [#] -- penalty for turning
static const float penaltyTurning = 1.05;

// [#] -- penalty for reversing
static const float penaltyReversing = 2.0;

// [#] -- penalty for change of direction
static const float penaltyCOD = 2.0;

// [m] -- dubins shot step size
static const float dubinsStepSize = 1;

// [m] -- dubins shot distance
static const float dubinsShotDistance = 2 * 2 * (r* r);

// ______________________
// DUBINS LOOKUP SPECIFIC

// [m] -- width of the dubinsArea / 2
static const int dubinsWidth = 15;
// [m²] -- area of the dubinsArea
static const int dubinsArea = dubinsWidth * dubinsWidth;


// _________________________
// COLLISION LOOKUP SPECIFIC

// [m] -- bounding box size length/width
static const int bbSize = std::ceil((sqrt(width * width + length* length) + 4) / cellSize);

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
}

#endif // CONSTANTS

