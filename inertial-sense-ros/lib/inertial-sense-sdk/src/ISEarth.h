/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef ISEARTH_H_
#define ISEARTH_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "ISMatrix.h"
#include "ISConstants.h"
#include "ISPose.h"

#define DEG2RAD_EARTH_RADIUS_F		111120.0f					// = DEG2RAD * earth_radius_in_meters
#define INV_DEG2RAD_EARTH_RADIUS_F	8.99928005759539236861e-6f	// = 1 / ( DEG2RAD * earth_radius_in_meters )

#define EARTH_RADIUS_F			6366707.01949371f				// = earth_radius_in_meters
#define INV_EARTH_RADIUS_F		1.5706706731410E-07f				// = 1 / earth_radius_in_meters

#if 0
typedef Vector2     Vector2_t;
typedef Vector3     Vector3_t;
typedef Vector4     Vector4_t;
typedef Vector3_t   Euler_t;        // phi, theta, psi (roll, pitch, yaw)
typedef Vector4_t   Quat_t;         // w, x, y, z
typedef Matrix2     Matrix2_t;
typedef Matrix3     Matrix3_t;
typedef Matrix4     Matrix4_t;
#else

#endif


#if (!defined (__cplusplus) && (!defined (inline)))
#       define inline __inline          // allow "inline" keyword to work in windows w/ c program
#endif


//_____ G L O B A L S ______________________________________________________

//_____ P R O T O T Y P E S ________________________________________________

/* 
 * Coordinate transformation from ECEF coordinates to latitude/longitude/altitude
 */
void ecef2lla(const double *Pe, double *LLA, const int Niter);

/*
 * Coordinate transformation from latitude/longitude/altitude to ECEF coordinates
 */
void lla2ecef(const double *LLA, double *Pe);

/*
 *  Find NED (north, east, down) from LLAref to LLA
 *
 *  lla[0] = latitude (rad)
 *  lla[1] = longitude (rad)
 *  lla[2] = msl altitude (m)
 */
void lla2ned( Vector3_t llaRef, Vector3_t lla, Vector3_t result );
void lla2ned_d( double llaRef[3], double lla[3], Vector3_t result );     // double precision

/*
 *  Find NED (north, east, down) from LLAref to LLA
 *
 *  lla[0] = latitude (degrees)
 *  lla[1] = longitude (degrees)
 *  lla[2] = msl altitude (m)
 */
void llaDeg2ned_d(double llaRef[3], double lla[3], Vector3_t result);

/*
 *  Find LLA of NED (north, east, down) from LLAref
 *
 *  lla[0] = latitude (rad)
 *  lla[1] = longitude (rad)
 *  lla[2] = msl altitude (m)
 */
void ned2lla( Vector3_t ned, Vector3_t llaRef, Vector3_t result );
void ned2lla_d( Vector3_t ned, double llaRef[3], double result[3] );     // double precision

/*
*  Find LLA of NED (north, east, down) from LLAref (WGS-84 standard)
*
*  lla[0] = latitude (degrees)
*  lla[1] = longitude (degrees)
*  lla[2] = msl altitude (m)
*/
void ned2llaDeg_d(Vector3_t ned, double llaRef[3], double result[3]);

/*
 *  Find Delta LLA of NED (north, east, down) from LLAref
 *
 *  lla[0] = latitude (rad)
 *  lla[1] = longitude (rad)
 *  lla[2] = msl altitude (m)
 */
void ned2DeltaLla(Vector3_t ned, Vector3 llaRef, Vector3 deltaLLA);
void ned2DeltaLla_d(Vector3_t ned, double llaRef[3], double deltaLLA[3]);

/*
*  Find Delta LLA of NED (north, east, down) from LLAref
*
*  lla[0] = latitude (degrees)
*  lla[1] = longitude (degrees)
*  lla[2] = msl altitude (m)
*/
void ned2DeltaLlaDeg_d(Vector3_t ned, double llaRef[3], double deltaLLA[3]);

// Convert LLA from radians to degrees
void lla_Rad2Deg_d(double result[3], double lla[3]);

// Convert LLA from degrees to radians
void lla_Deg2Rad_d(double result[3], double lla[3]);
void lla_Deg2Rad_d2(double result[3], double lat, double lon, double alt);

/*
 *  Find msl altitude based on barometric pressure
 *  https://en.wikipedia.org/wiki/Atmospheric_pressure
 *
 *  baroKPa = (kPa) barometric pressure in kilopascals
 *  return = (m) MSL altitude in meters
 */
f_t baro2msl( f_t pKPa );

/*
 *  Find linear distance between lat,lon,alt (rad,rad,m) coordinates.
 *
 *  return = (m) distance in meters
 */
f_t llaRadDistance( double lla1[3], double lla2[3] );
f_t llaDegDistance( double lla1[3], double lla2[3] );

/*
 *  Check if lat,lon,alt (deg,deg,m) coordinates are valid.
 *
 *  return 0 on success, -1 on failure.
 */
int llaDegValid( double lla[3] );

/* 
 * IGF-80 gravity model with WGS-84 ellipsoid refinement 
*/
float gravity_igf80(double lat, double alt);

#ifdef __cplusplus
}
#endif

#endif /* ISEARTH_H_ */
