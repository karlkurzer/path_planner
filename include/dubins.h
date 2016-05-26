/*!
  \file dubins.h
   \brief A dubins path class for finding analytical solutions to the problem of the shortest path.

  It has been copied from https://github.com/AndrewWalker/Dubins-Curves/ under the WTFPL. Please refer to the author in case of questions.
  \author Andrew Walker
*/
// Copyright (c) 2008-2014, Andrew Walker
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
#ifndef DUBINS_H
#define DUBINS_H

// Path types
#define LSL (0)
#define LSR (1)
#define RSL (2)
#define RSR (3)
#define RLR (4)
#define LRL (5)

// Error codes
#define EDUBOK        (0)   // No error
#define EDUBCOCONFIGS (1)   // Colocated configurations
#define EDUBPARAM     (2)   // Path parameterisitation error
#define EDUBBADRHO    (3)   // the rho value is invalid
#define EDUBNOPATH    (4)   // no connection between configurations with this word

namespace HybridAStar {

// The various types of solvers for each of the path types
typedef int (*DubinsWord)(double, double, double, double* );

// A complete list of the possible solvers that could give optimal paths
extern DubinsWord dubins_words[];

typedef struct
{
    double qi[3];       // the initial configuration
    double param[3];    // the lengths of the three segments
    double rho;         // model forward velocity / model angular velocity
    int type;           // path type. one of LSL, LSR, ...
} DubinsPath;

/**
 * Callback function for path sampling
 *
 * @note the q parameter is a configuration
 * @note the t parameter is the distance along the path
 * @note the user_data parameter is forwarded from the caller
 * @note return non-zero to denote sampling should be stopped
 */
typedef int (*DubinsPathSamplingCallback)(double q[3], double t, void* user_data);

/**
 * Generate a path from an initial configuration to
 * a target configuration, with a specified maximum turning
 * radii
 *
 * A configuration is (x, y, theta), where theta is in radians, with zero
 * along the line x = 0, and counter-clockwise is positive
 *
 * @param q0    - a configuration specified as an array of x, y, theta
 * @param q1    - a configuration specified as an array of x, y, theta
 * @param rho   - turning radius of the vehicle (forward velocity divided by maximum angular velocity)
 * @param path  - the resultant path
 * @return      - non-zero on error
 */
int dubins_init( double q0[3], double q1[3], double rho, DubinsPath* path);

/**
 * Calculate the length of an initialised path
 *
 * @param path - the path to find the length of
 */
double dubins_path_length( DubinsPath* path );

/**
 * Extract an integer that represents which path type was used
 *
 * @param path    - an initialised path
 * @return        - one of LSL, LSR, RSL, RSR, RLR or LRL (ie/ 0-5 inclusive)
 */
int dubins_path_type( DubinsPath * path );

/**
 * Calculate the configuration along the path, using the parameter t
 *
 * @param path - an initialised path
 * @param t    - a length measure, where 0 <= t < dubins_path_length(path)
 * @param q    - the configuration result
 * @returns    - non-zero if 't' is not in the correct range
 */
int dubins_path_sample( DubinsPath* path, double t, double q[3]);

/**
 * Walk along the path at a fixed sampling interval, calling the
 * callback function at each interval
 *
 * @param path      - the path to sample
 * @param cb        - the callback function to call for each sample
 * @param user_data - optional information to pass on to the callback
 * @param stepSize  - the distance along the path for subsequent samples
 */
int dubins_path_sample_many( DubinsPath* path, DubinsPathSamplingCallback cb, double stepSize, void* user_data );

/**
 * Convenience function to identify the endpoint of a path
 *
 * @param path - an initialised path
 * @param q    - the configuration result
 */
int dubins_path_endpoint( DubinsPath* path, double q[3] );

/**
 * Convenience function to extract a subset of a path
 *
 * @param path    - an initialised path
 * @param t       - a length measure, where 0 < t < dubins_path_length(path)
 * @param newpath - the resultant path
 */
int dubins_extract_subpath( DubinsPath* path, double t, DubinsPath* newpath );

// Only exposed for testing purposes
int dubins_LSL( double alpha, double beta, double d, double* outputs );
int dubins_RSR( double alpha, double beta, double d, double* outputs );
int dubins_LSR( double alpha, double beta, double d, double* outputs );
int dubins_RSL( double alpha, double beta, double d, double* outputs );
int dubins_LRL( double alpha, double beta, double d, double* outputs );
int dubins_RLR( double alpha, double beta, double d, double* outputs );

}
#endif // DUBINS_H
