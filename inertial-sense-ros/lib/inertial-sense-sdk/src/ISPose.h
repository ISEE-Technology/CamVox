/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef POSE_H_
#define POSE_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "ISMatrix.h"
#include "ISConstants.h"

//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

//typedef f_t Quat_t[4];		// [w, x, y, z]
//typedef f_t Euler_t[3];

#if 1
typedef Vector2     Vector2_t;
typedef Vector3     Vector3_t;
typedef Vector4     Vector4_t;
typedef Vector3_t   Euler_t;        // phi, theta, psi (roll, pitch, yaw)
typedef Vector4_t   Quat_t;         // [w, x, y, z]
typedef f_t			q_t;
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
 * Initialize Quaternion q = [w, x, y, z]
 */
void quat_init( Quat_t q );

/* Quaternion Conjugate: q* = [ w, -x, -y, -z ] of quaterion q = [ w, x, y, z ] 
 * Rotation in opposite direction.
 */
void quatConj( Quat_t result, const Quat_t q );

/* 
* Product of two Quaternions.  Order of q1 and q2 matters (same as applying two successive DCMs)!!!  
* Combines two quaternion rotations into one rotation.
* result = q1 * q2. 
* Reference: http://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
*/
void mul_Quat_Quat( Quat_t result, const Quat_t q1, const Quat_t q2 );

/*
* Product of two Quaternions.  Order of q1 and q2 matters (same as applying two successive DCMs)!!!
* Combines two quaternion rotations into one rotation.
* result = quatConj(q1) * q2.
* Reference: http://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
*/
void mul_ConjQuat_Quat( Quat_t result, const Quat_t qc, const Quat_t q2 );

/*
* Product of two Quaternions.  Order of q1 and q2 matters (same as applying two successive DCMs)!!!
* Combines two quaternion rotations into one rotation.
* result = q1 * quatConj(q2)
* Reference: http://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
*/
void mul_Quat_ConjQuat(Quat_t result, const Quat_t q1, const Quat_t qc);

/*
 * Division of two Quaternions.  Order matters!!!
 * result = q1 / q2. 
 * Reference: http://www.mathworks.com/help/aeroblks/quaterniondivision.html
 */
void div_Quat_Quat( Quat_t result, const Quat_t q1, const Quat_t q2 );

/* 
 * Quaternion rotation from vector v1 to vector v2.
 */
void quat_Vec3_Vec3( Quat_t result, const Vector3_t v1, const Vector3_t v2 );

/* Computationally simple means to apply quaternion rotation to a vector.
 * Requires quaternion be normalized first.  
 * If quaternion describes current attitude, then rotation is body frame -> reference frame.
 */
void quatRot( Vector3_t result, const Quat_t q, const Vector3_t v );

/* Computationally simple means to apply quaternion conjugate (opposite) rotation to a vector
 * Requires quaternion be normalized first
 * If quaternion describes current attitude, then rotation is reference frame -> body frame.
 */
void quatConjRot( Vector3_t result, const Quat_t q, const Vector3_t v );

/*
 * This will convert from quaternions to euler angles
 * q(W,X,Y,Z) -> euler(phi,theta,psi) (rad)
 *
 * Reference: http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 */
void quat2euler(const Quat_t q, Euler_t theta);
void quat2phiTheta(const Quat_t q, f_t *phi, f_t *theta);
void quat2psi(const Quat_t q, f_t *psi);

/*
 * This will convert from euler angles to quaternion vector
 * euler(phi,theta,psi) (rad) -> q(W,X,Y,Z)
 */
void euler2quat(const Euler_t euler, Quat_t q );


/*
 * Quaternion rotation to NED with respect to ECEF at specified LLA
 */
// void quatEcef2Ned(Vector4 Qe2n, const Vector3d lla);

/* Attitude quaternion for NED frame in ECEF */
void quat_ecef2ned(float lat, float lon, float *qe2n);

/*
* Convert ECEF quaternion to NED euler at specified ECEF
*/
void qe2b2EulerNedEcef(Vector3 theta, const Vector4 qe2b, const Vector3d ecef);
void qe2b2EulerNedLLA(Vector3 eul, const Vector4 qe2b, const Vector3d lla);

/*
 * This will construct a direction cosine matrix from
 * the psi angle - rotates from NE to body frame
 *
 * body = tBL(2,2)*NE
 *
 */
void psiDCM(const f_t psi, Matrix2_t m);

/*
* This will extract the psi euler angle from a direction cosine matrix in the
* standard rotation sequence, for either a 2x2 or 3x3 DCM matrix.
* [phi][theta][psi] from NED to body frame
*
* body = tBL(2,2)*NE
* body = tBL(3,3)*NED
*
* reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
*/
f_t DCMpsi(const f_t *m );

/*
 * This will construct a direction cosine matrix from
 * euler angles in the standard rotation sequence
 * [phi][theta][psi] from NED to body frame
 *
 * body = tBL(3,3)*NED
 *
 * Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
 */
//const Matrix<3,3> eulerDCM( const Vector<3> & euler )
void eulerDCM(const Euler_t euler, Matrix3_t m );
// Only use phi and theta (exclude psi) in rotation
void phiThetaDCM(const Euler_t euler, Matrix3_t m );

/*
 * This will construct the transpose matrix of
 * the direction cosine matrix from
 * euler angles in the standard rotation sequence
 * [phi][theta][psi] from NED to body frame
 *
 * body = tBL(3,3)*NED
 *
 * reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
 */
void eulerDCM_Trans(const Euler_t euler, Matrix3_t m );

/*
 * This will extract euler angles from a direction cosine matrix in the
 * standard rotation sequence.
 * [phi][theta][psi] from NED to body frame
 *
 * body = tBL(3,3)*NED
 *
 * Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
 */
void DCMeuler(const Matrix3_t m, Euler_t euler);


/*
 * This will construct a direction cosine matrix from
 * quaternions in the standard rotation  sequence
 * [phi][theta][psi] from NED to body frame
 *
 * body = tBL(3,3)*NED
 * q(4,1)
 *
 * Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
 */
void quatDCM(const Quat_t q, Matrix3_t mat);
void quatdDCM(const Vector4d q, Matrix3_t mat);

/*
 * This will construct a quaternion from direction 
 * cosine matrix in the standard rotation sequence
 * [phi][theta][psi] from NED to body frame
 *
 * body = tBL(3,3)*NED
 * q(4,1)
 *
 * Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
 */
void DCMquat(const Matrix3_t mat, Quat_t q);

/*
 * This will construct the euler omega-cross matrix
 * wx(3,3)
 * p, q, r (rad/sec)
 */
void eulerWx(const Euler_t euler, Matrix3_t mat);

/*
 * This will construct the quaternion omega matrix
 * W(4,4)
 * p, q, r (rad/sec)
 */
void quatW(const Euler_t euler, Matrix4_t mat);

/*
*   Convert quaternion to rotation axis (and angle).  Quaternion must be normalized.
*/
void quatRotAxis(const Quat_t q, Vector3_t pqr);

/*
 *  Compute the derivative of the Euler_t angle psi with respect
 * to the quaternion Q.  The result is a row vector
 *
 * d(psi)/d(q0)
 * d(psi)/d(q1)
 * d(psi)/d(q2)
 * d(psi)/d(q3)
 */
void dpsi_dq(const Quat_t q, Quat_t dq);

/*
 * NED to Euler_t
 */
void nedEuler(const Vector3_t ned, Euler_t e);

/*
 * Euler_t to NED
 */
void eulerNed(const Euler_t e, Vector3_t ned);

/*
 * Rotate theta eulers from body frame to reference frame by eulers, in order: phi, theta, psi
 */
void eulerBodyToReference(const  Euler_t e, const  Euler_t rot, Euler_t result);

/*
 * Rotate theta eulers from reference frame to body frame by eulers, in order: psi, theta, phi
 */
void eulerReferenceToBody(const  Euler_t e, const Euler_t rot, Euler_t result);

/*
 * Rotate vector from body frame to reference frame by euler angles, in order: phi, theta, psi
 */
void vectorBodyToReference(const  Vector3_t v, const Euler_t rot, Vector3_t result);

/*
 * Rotate vector from reference frame to body frame by euler angles, in order: psi, theta, phi
 */
void vectorReferenceToBody(const  Vector3_t v, const Euler_t rot, Vector3_t result);


#ifdef __cplusplus
}
#endif

#endif /* POSE_H_ */
