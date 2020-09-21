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

/*!
    \brief The namespace that wraps the entire project
    \namespace HybridAStar
*/

namespace HybridAStar {
/*!
    \brief The namespace that wraps constants.h
    \namespace Constants
*/
namespace Constants {
// _________________
// CONFIG FLAGS

/// A flag for additional debugging output via `std::cout`
static const bool coutDEBUG = false;
/// A flag for the mode (true = manual; false = dynamic). Manual for static map or dynamic for dynamic map.
static const bool manual = true;
/// A flag for the visualization of 3D nodes (true = on; false = off)
static const bool visualization = false && manual;
/// A flag for the visualization of 2D nodes (true = on; false = off)
static const bool visualization2D = false && manual;
/// A flag to toggle reversing (true = on; false = off)
static const bool reverse = true;
/// A flag to toggle the connection of the path via Dubin's shot (true = on; false = off)
static const bool dubinsShot = true;
/// A flag to toggle the Dubin's heuristic, this should be false, if reversing is enabled (true = on; false = off)
static const bool dubins = false;
/*!
   \var static const bool dubinsLookup
   \brief A flag to toggle the Dubin's heuristic via lookup, potentially speeding up the search by a lot
   \todo not yet functional
*/
static const bool dubinsLookup = false && dubins;
/// A flag to toggle the 2D heuristic (true = on; false = off)
static const bool twoD = true;

// _________________
// GENERAL CONSTANTS

/// [#] --- Limits the maximum search depth of the algorithm, possibly terminating without the solution
static const int iterations = 30000;
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
  \brief [m] --- The tie breaker breaks ties between nodes expanded in the same cell


  As the cost-so-far are bigger than the cost-to-come it is reasonbale to believe that the algorithm would prefer the predecessor rather than the successor.
  This would lead to the fact that the successor would never be placed and the the one cell could only expand one node. The tieBreaker artificially increases the cost of the predecessor
  to allow the successor being placed in the same cell.
*/
static const float tieBreaker = 0.01;

// ___________________
// HEURISTIC CONSTANTS

/// [#] --- A factor to ensure admissibility of the holonomic with obstacles heuristic
static const float factor2D = sqrt(5) / sqrt(2) + 1;
/// [#] --- A movement cost penalty for turning (choosing non straight motion primitives)
static const float penaltyTurning = 1.05;
/// [#] --- A movement cost penalty for reversing (choosing motion primitives > 2)
static const float penaltyReversing = 2.0;
/// [#] --- A movement cost penalty for change of direction (changing from primitives < 3 to primitives > 2)
static const float penaltyCOD = 2.0;
/// [m] --- The distance to the goal when the analytical solution (Dubin's shot) first triggers
static const float dubinsShotDistance = 100;
/// [m] --- The step size for the analytical solution (Dubin's shot) primarily relevant for collision checking
static const float dubinsStepSize = 1;


// ______________________
// DUBINS LOOKUP SPECIFIC

/// [m] --- The width of the dubinsArea / 2 for the analytical solution (Dubin's shot)
static const int dubinsWidth = 15;
/// [m] --- The area of the lookup for the analytical solution (Dubin's shot)
static const int dubinsArea = dubinsWidth * dubinsWidth;


// _________________________
// COLLISION LOOKUP SPECIFIC

/// [m] -- The bounding box size length and width to precompute all possible headings
static const int bbSize = std::ceil((sqrt(width * width + length* length) + 4) / cellSize);
/// [#] --- The sqrt of the number of discrete positions per cell
static const int positionResolution = 10;
/// [#] --- The number of discrete positions per cell
static const int positions = positionResolution * positionResolution;
/// A structure describing the relative position of the occupied cell based on the center of the vehicle
struct relPos {
  /// the x position relative to the center
  int x;
  /// the y position relative to the center
  int y;
};
/// A structure capturing the lookup for each theta configuration
struct config {
  /// the number of cells occupied by this configuration of the vehicle
  int length;
  /*!
     \var relPos pos[64]
     \brief The maximum number of occupied cells
     \todo needs to be dynamic
  */
  relPos pos[64];
};

// _________________
// SMOOTHER SPECIFIC
/// [m] --- The minimum width of a safe road for the vehicle at hand
static const float minRoadWidth = 2;

// ____________________________________________
// COLOR DEFINITIONS FOR VISUALIZATION PURPOSES
/// A structure to express colors in RGB values
struct color {
  /// the red portion of the color
  float red;
  /// the green portion of the color
  float green;
  /// the blue portion of the color
  float blue;
};
/// A definition for a color used for visualization
static constexpr color teal = {102.f / 255.f, 217.f / 255.f, 239.f / 255.f};
/// A definition for a color used for visualization
static constexpr color green = {166.f / 255.f, 226.f / 255.f, 46.f / 255.f};
/// A definition for a color used for visualization
static constexpr color orange = {253.f / 255.f, 151.f / 255.f, 31.f / 255.f};
/// A definition for a color used for visualization
static constexpr color pink = {249.f / 255.f, 38.f / 255.f, 114.f / 255.f};
/// A definition for a color used for visualization
static constexpr color purple = {174.f / 255.f, 129.f / 255.f, 255.f / 255.f};
}
}

#endif // CONSTANTS

