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
    \brief The namespace that wraps constants.h
    \namespace constants
*/
namespace Constants {
// _________________
// CONFIG FLAGS

/// A flag for additional debugging output via `std::cout`
static const bool coutDEBUG = 0;
/// A flag for the mode (true = manual; false = dynamic). Manual for static map or dynamic for dynamic map.
static const bool manual = 1;
/// A flag for the visualization of 3D nodes (true = on; false = off)
static const bool visualization = 0 * manual;
/// A flag for the visualization of 2D nodes (true = on; false = off)
static const bool visualization2D = 0 * manual;
/// A flag to toggle reversing (true = on; false = off)
static const bool reverse = true;
/// A flag to toggle the connection of the path via Dubin's shot (true = on; false = off)
static const bool dubinsShot = true;
/// A flag to toggle the Dubin's heuristic, this should be false, if reversing is enabled (true = on; false = off)
static const bool dubins = false;
/*!
   \var bool dubinsLookup
   \brief A flag to toggle the Dubin's heuristic via lookup, potentially speeding up the search by a lot
   \todo not yet functional
*/
static const bool dubinsLookup = false * dubins;
/// A flag to toggle the 2D heuristic (true = on; false = off)
static const bool twoD = true;

// _________________
// GENERAL CONSTANTS

/// [#] --- Limits the maximum search depth of the algorithm, possibly terminating without the solution
static const int iterations = 10000;
/// [m] --- Uniformly adds a padding around the vehicle
static const double bloating = 0;
/// [m] --- The width of the vehicle
static const double width = 1.75 + 2 * bloating;
/// [m] --- The length of the vehicle
static const double length = 2.65 + 2 * bloating;
/// [m] --- The minimum turning radius of the vehicle
static const float r = 6;
/// [m] --- The number of discretizations in heading
static const int headings = 72;
/// [°] --- The discretization value of the heading (goal condition)
static const float deltaHeadingDeg = 360 / (float)headings;
/// [c*M_PI] --- The discretization value of heading (goal condition)
static const float deltaHeadingRad = 2 * M_PI / (float)headings;
/// [c*M_PI] --- The heading part of the goal condition
static const float deltaHeadingNegRad = 2 * M_PI - deltaHeadingRad;
/// [m] --- The cell size of the 2D grid of the world
static const float cellSize = 1;
/*!
  \var float tieBreaker
  \brief [m] --- The tie breaker breaks ties between nodes expanded in the same cell


  As the cost-so-far are bigger than the cost-to-come it is reasonbale to believe that the algorithm would prefer the predecessor rather than the successor.
  This would lead to the fact that the successor would never be placed and the the one cell could only expand one node. The tieBreaker artificially increases the cost of the predecessor
  to allow the successor being placed in the same cell.
*/
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

