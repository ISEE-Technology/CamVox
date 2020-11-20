/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#define _MATH_DEFINES_DEFINED
#include <math.h>

// #include "misc/debug.h"
// #include "ISConstants.h"

#include "ISEarth.h"

//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

#define C_NEG_MG_DIV_KT0_F			-1.18558314779367E-04f		// - (M * g)  / (K * T0)
#define C_NEG_KT0_DIV_MG_F			-8.43466779922578000E+03f	// - (K * T0) / (M * g)

//_____ G L O B A L S ______________________________________________________

//_____ L O C A L   P R O T O T Y P E S ____________________________________

//_____ F U N C T I O N S __________________________________________________
// static const double Ra = 6378137.0;			// (m) Earth equatorial radius
// static const double Rb = 6356752.31424518;	// (m) Earth polar radius Rb = Ra * (1-f)   (from flattening, f = 1/298.257223563)

#define POWA2	40680631590769.000	// = pow(6378137.0,2)
#define POWB2	40408299984661.453	// = pow(6356752.31424518,2)
#define POWA2_F	40680631590769.000f	// = pow(6378137.0,2)
#define POWB2_F	40408299984661.453f	// = pow(6356752.31424518,2)

#define ONE_MINUS_F 0.996647189335253 // (1 - f), where f = 1.0 / 298.257223563 is Earth flattening
#define E_SQ  0.006694379990141 // e2 = 1 - (1-f)*(1-f) - square of first eccentricity
#define REQ 6378137.0         // Re - Equatorial radius, m
#define REP 6356752.314245179 // Rp - Polar radius, m
#define E2xREQ 42697.67270717795 // e2 * Re
#define E2xREQdivIFE 42841.31151331153 // e2 * Re / (1 -f)
#define GEQ 9.7803253359        // Equatorial gravity
#define K_GRAV 0.00193185265241 // defined gravity constants
#define K3_GRAV 3.0877e-6       // 
#define K4_GRAV 4.0e-9          //
#define K5_GRAV 7.2e-14         //

/* Coordinate transformation from ECEF coordinates to latitude/longitude/altitude */
void ecef2lla(const double *Pe, double *LLA, const int Niter)
{
    int i;
    double s, Rn, sinmu, beta;

    // Earth equatorial radius
    // Re = 6378137; // m
    // Square of first eccentricity
    // e2 = 1 - (1-f)*(1-f);

    // Longitude
    LLA[1] = atan2(Pe[1], Pe[0]);

    // Latitude computation using Bowring's method, 
    // which typically converges after 2 or 3 iterations
    s = sqrt(Pe[0] * Pe[0] + Pe[1] * Pe[1]);
    beta = atan2(Pe[2], ONE_MINUS_F * s); // reduced latitude, initial guess

    // Precompute these values to speed-up computation
    // B = e2 * Re; // >>> this is now E2xREQ
    // A = e2 * Re / (1 - f); // >>> this is now E2xREQdivIFE
    for (i = 0; i < Niter; i++) {
        // iterative latitude computation
        LLA[0] = atan2(Pe[2] + E2xREQdivIFE * pow(sin(beta), 3), s - E2xREQ * pow(cos(beta), 3));
        beta   = atan(ONE_MINUS_F * tan(LLA[0]));
    }

    // Radius of curvature in the vertical prime
    sinmu = sin(LLA[0]);
    Rn = REQ / sqrt(1.0 - E_SQ * sinmu * sinmu);

    // Altitude above planetary ellipsoid
    LLA[2] = s * cos(LLA[0]) + (Pe[2] + E_SQ * Rn * sinmu) * sinmu - Rn;
}


/* Coordinate transformation from latitude/longitude/altitude to ECEF coordinates */
void lla2ecef(const double *LLA, double *Pe)
{
    //double e = 0.08181919084262;  // Earth first eccentricity: e = sqrt((R^2-b^2)/R^2);
    double Rn, Smu, Cmu, Sl, Cl;

    /* Earth equatorial and polar radii 
      (from flattening, f = 1/298.257223563; */
    // R = 6378137; // m
    // Earth polar radius b = R * (1-f)
    // b = 6356752.31424518;

    Smu = sin(LLA[0]);
    Cmu = cos(LLA[0]);
    Sl  = sin(LLA[1]);
    Cl  = cos(LLA[1]);
    
    // Radius of curvature at a surface point:
    Rn = REQ / sqrt(1.0 - E_SQ * Smu * Smu);

    Pe[0] = (Rn + LLA[2]) * Cmu * Cl;
    Pe[1] = (Rn + LLA[2]) * Cmu * Sl;
    Pe[2] = (Rn * POWB2 / POWA2 + LLA[2]) * Smu;
}


/*
 *  Find NED (north, east, down) from LLAref to LLA (WGS-84 standard)
 *
 *  lla[0] = latitude (rad)
 *  lla[1] = longitude (rad)
 *  lla[2] = msl altitude (m)
 */
void lla2ned(Vector3_t llaRef, Vector3_t lla, Vector3_t result)
{
    Vector3_t deltaLLA;
    deltaLLA[0] = lla[0] - llaRef[0];
    deltaLLA[1] = lla[1] - llaRef[1];
    deltaLLA[2] = lla[2] - llaRef[2];
    
	// Handle longitude wrapping 
	UNWRAP_F32(deltaLLA[1]);

    // Find NED
	result[0] =  deltaLLA[0] * EARTH_RADIUS_F;
	result[1] =  deltaLLA[1] * EARTH_RADIUS_F * _COS( llaRef[0] );
	result[2] = -deltaLLA[2];
}


/*
 *  Find NED (north, east, down) from LLAref to LLA (WGS-84 standard)
 *
 *  lla[0] = latitude (rad)
 *  lla[1] = longitude (rad)
 *  lla[2] = msl altitude (m)
 */
void lla2ned_d(double llaRef[3], double lla[3], Vector3_t result)
{
    Vector3_t deltaLLA;
    deltaLLA[0] = (f_t)(lla[0] - llaRef[0]);
    deltaLLA[1] = (f_t)(lla[1] - llaRef[1]);
    deltaLLA[2] = (f_t)(lla[2] - llaRef[2]);

	// Handle longitude wrapping in radians
	UNWRAP_F32(deltaLLA[1]);
    
    // Find NED
	result[0] =  deltaLLA[0] * EARTH_RADIUS_F;
	result[1] =  deltaLLA[1] * EARTH_RADIUS_F * _COS( ((f_t)llaRef[0]) );
	result[2] = -deltaLLA[2];
}

/*
 *  Find NED (north, east, down) from LLAref to LLA (WGS-84 standard)
 *
 *  lla[0] = latitude (deg)
 *  lla[1] = longitude (deg)
 *  lla[2] = msl altitude (m)
 */
void llaDeg2ned_d(double llaRef[3], double lla[3], Vector3_t result)
{
    Vector3_t deltaLLA;
    deltaLLA[0] = (f_t)(lla[0] - llaRef[0]);
    deltaLLA[1] = (f_t)(lla[1] - llaRef[1]);
    deltaLLA[2] = (f_t)(lla[2] - llaRef[2]);

	// Handle longitude wrapping 
	UNWRAP_DEG_F32(deltaLLA[1]);
    
    // Find NED
	result[0] =  deltaLLA[0] * DEG2RAD_EARTH_RADIUS_F;
	result[1] =  deltaLLA[1] * DEG2RAD_EARTH_RADIUS_F * _COS( ((f_t)llaRef[0]) * C_DEG2RAD_F );
	result[2] = -deltaLLA[2];
}


/*
 *  Find LLA of NED (north, east, down) from LLAref (WGS-84 standard)
 *
 *  lla[0] = latitude (rad)
 *  lla[1] = longitude (rad)
 *  lla[2] = msl altitude (m)
 */
void ned2lla(Vector3_t ned, Vector3_t llaRef, Vector3_t result)
{
    Vector3 deltaLLA;
    ned2DeltaLla( ned, llaRef, deltaLLA );
    
    // Find LLA
    result[0] = llaRef[0] + deltaLLA[0];
    result[1] = llaRef[1] + deltaLLA[1];
    result[2] = llaRef[2] + deltaLLA[2];
}


/*
 *  Find LLA of NED (north, east, down) from LLAref (WGS-84 standard)
 *
 *  lla[0] = latitude (rad)
 *  lla[1] = longitude (rad)
 *  lla[2] = msl altitude (m)
 */
void ned2lla_d(Vector3_t ned, double llaRef[3], double result[3])
{
    double deltaLLA[3];
    ned2DeltaLla_d( ned, llaRef, deltaLLA );
    
    // Find LLA
	result[0] = llaRef[0] + deltaLLA[0];
	result[1] = llaRef[1] + deltaLLA[1];
	result[2] = llaRef[2] + deltaLLA[2];
}


/*
*  Find LLA of NED (north, east, down) from LLAref (WGS-84 standard)
*
*  lla[0] = latitude (degrees)
*  lla[1] = longitude (degrees)
*  lla[2] = msl altitude (m)
*/
void ned2llaDeg_d(Vector3_t ned, double llaRef[3], double result[3])
{
	double deltaLLA[3];
	ned2DeltaLlaDeg_d(ned, llaRef, deltaLLA);

	// Find LLA
	result[0] = llaRef[0] + deltaLLA[0];
	result[1] = llaRef[1] + deltaLLA[1];
	result[2] = llaRef[2] + deltaLLA[2];
}


/*
 *  Find msl altitude based on barometric pressure
 *  https://en.wikipedia.org/wiki/Atmospheric_pressure
 *
 *  baroKPa = (kPa) barometric pressure in kilopascals
 *  return = (m) msl altitude in meters
 */
f_t baro2msl( f_t pKPa )
{
	if( pKPa <= _ZERO )
		return _ZERO;
	else
		return (f_t)C_NEG_KT0_DIV_MG_F * _LOG( pKPa * (f_t)C_INV_P0_KPA_F );
}


/*
 *  Find linear distance between lat,lon,alt (rad,rad,m) coordinates.
 *
 *  return = (m) distance in meters
 */
f_t llaRadDistance( double lla1[3], double lla2[3] )
{
	Vector3_t ned;
	
	lla2ned_d( lla1, lla2, ned );

	return _SQRT( ned[0]*ned[0] + ned[1]*ned[1] + ned[2]*ned[2] );
}

/*
 *  Find linear distance between lat,lon,alt (deg,deg,m) coordinates.
 *
 *  return = (m) distance in meters
 */
f_t llaDegDistance( double lla1[3], double lla2[3] )
{
	Vector3_t ned;
	
	llaDeg2ned_d( lla1, lla2, ned );

	return _SQRT( ned[0]*ned[0] + ned[1]*ned[1] + ned[2]*ned[2] );
}

void ned2DeltaLla(Vector3_t ned, Vector3 llaRef, Vector3 deltaLLA)
{
	deltaLLA[0] =  ned[0] * INV_EARTH_RADIUS_F;
	deltaLLA[1] =  ned[1] * INV_EARTH_RADIUS_F / _COS(((f_t)llaRef[0]));
	deltaLLA[2] = -ned[2];
}

void ned2DeltaLla_d(Vector3_t ned, double llaRef[3], double deltaLLA[3])
{
	deltaLLA[0] = (double)( ned[0] * INV_EARTH_RADIUS_F);
	deltaLLA[1] = (double)( ned[1] * INV_EARTH_RADIUS_F / _COS(((f_t)llaRef[0])) );
	deltaLLA[2] = (double)(-ned[2]);
}

void ned2DeltaLlaDeg_d(Vector3_t ned, double llaRef[3], double deltaLLA[3])
{
	deltaLLA[0] = (double)(ned[0] * INV_EARTH_RADIUS_F * C_RAD2DEG_F);
	deltaLLA[1] = (double)(ned[1] * INV_EARTH_RADIUS_F * C_RAD2DEG_F / _COS( (((f_t)llaRef[0])*C_DEG2RAD_F) ) );
	deltaLLA[2] = (double)(-ned[2]);
}

// Convert LLA from radians to degrees
void lla_Rad2Deg_d(double result[3], double lla[3])
{
	result[0] = C_RAD2DEG * lla[0];
	result[1] = C_RAD2DEG * lla[1];
	result[2] = lla[2];
}

// Convert LLA from degrees to radians
void lla_Deg2Rad_d(double result[3], double lla[3])
{
	result[0] = C_DEG2RAD * lla[0];
	result[1] = C_DEG2RAD * lla[1];
	result[2] = lla[2];
}

void lla_Deg2Rad_d2(double result[3], double lat, double lon, double alt)
{
	result[0] = C_DEG2RAD * lat;
	result[1] = C_DEG2RAD * lon;
	result[2] = alt;
}

/*
 *  Check if lat,lon,alt (deg,deg,m) coordinates are valid.
 *
 *  return 1 on success, 0 on failure.
 */
int llaDegValid( double lla[3] )
{
    if( (lla[0]<-90.0)		|| (lla[0]>90.0) ||			// Lat
        (lla[1]<-180.0)		|| (lla[1]>180.0) ||		// Lon
        (lla[2]<-10000.0)   || (lla[2]>1000000.0) )		// Alt: -10 to 1,000 kilometers
    {    // Invalid
        return 0;
    }
    else
    {    // Valid
        return 1;
    }
}

/* IGF-80 gravity model with WGS-84 ellipsoid refinement */
float gravity_igf80(double lat, double alt)
{
    double g0, sinmu2;

    // Equatorial gravity
    //float ge = 9.7803253359f;
    // Defined constant k = (b*gp - a*ge) / a / ge;
    //double k = 0.00193185265241;
    // Square of first eccentricity e^2 = 1 - (1 - f)^2 = 1 - (b/a)^2;
    //double e2 = 0.00669437999013;

    sinmu2 = sin(lat) * sin(lat);
    g0 = GEQ * (1.0 + K_GRAV * sinmu2) / sqrt(1.0 - E_SQ * sinmu2);

    // Free air correction
    return (float)( g0 - (K3_GRAV - K4_GRAV * sinmu2 - K5_GRAV * alt) * alt );
}

