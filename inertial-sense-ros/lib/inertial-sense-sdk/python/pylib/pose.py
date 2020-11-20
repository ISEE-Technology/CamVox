'''
Created 9/4/2013

Author: Walt Johnson
'''
from __future__ import print_function

from numpy import sin, cos, arccos, arcsin, arctan2
from numpy import r_, c_
from numpy import dot
from numpy import pi
import numpy as np
from tqdm import tqdm

# import pylib.plotTools as pt


def quatInit():
    q = np.zeros(4)
    q[0] = 1.0
    return q

# Quaternion Conjugate: q* = [ w, -x, -y, -z ] of quaterion q = [ w, x, y, z ] 
# Rotation in opposite direction.
def quatConj( q ):
    return r_[ q[0], -q[1], -q[2], -q[3] ]

# Quaternion Inverse: q^-1 = q* / |q|^2
def quatInv( q ):
    return quatConj(q)/np.square(np.linalg.norm(q))


#  * Product of two Quaternions.  Order of q1 and q2 matters (same as applying two successive DCMs)!!!  
#  * Concatenates two quaternion rotations into one.
#  * result = q1 * q2 
#  * References: 
#  *    http://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
#  *    http://physicsforgames.blogspot.com/2010/02/quaternions.html
def mul_Quat_Quat( q1, q2 ):
    result = np.zeros(4)
    
    result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    result[1] = q1[0]*q2[1] + q1[1]*q2[0] - q1[2]*q2[3] + q1[3]*q2[2];
    result[2] = q1[0]*q2[2] + q1[1]*q2[3] + q1[2]*q2[0] - q1[3]*q2[1];
    result[3] = q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1] + q1[3]*q2[0];
    return result


#  * Division of two Quaternions.  Order matters!!!
#  * result = q1 / q2. 
#  * Reference: http://www.mathworks.com/help/aeroblks/quaterniondivision.html
def div_Quat_Quat( q1, q2 ):
    result = np.zeros(4)
    d = 1.0 / (q1[0]*q1[0] + q1[1]*q1[1] + q1[2]*q1[2] + q1[3]*q1[3])

    result[0] = (q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]) * d
    result[1] = (q1[0]*q2[1] - q1[1]*q2[0] - q1[2]*q2[3] + q1[3]*q2[2]) * d
    result[2] = (q1[0]*q2[2] + q1[1]*q2[3] - q1[2]*q2[0] - q1[3]*q2[1]) * d
    result[3] = (q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1] - q1[3]*q2[0]) * d
    return result


# Quaternion rotation from vector v1 to vector v2.
# Reference: 
def quat_Vec3_Vec3( v1, v2 ):
#     Vector3_t w1, w2;
    
    # Normalize input vectors
    w1 = v1 / np.linalg.norm(v1)
    w2 = v2 / np.linalg.norm(v2)
 
    qResult = np.zeros(4)
    # q[1:3]
    qResult[1:4] = np.cross(w1, w2)
    
    # q[0]
    qResult[0] = np.sqrt( np.square( np.dot(w1,w1) ) ) + np.dot(w1, w2)

    # Normalize quaternion
    qResult = qResult / np.linalg.norm(qResult)
    return qResult


def normalize( v ):
    return v / np.linalg.norm(v)


# Computationally simple means to apply quaternion rotation to a vector
# Requires quaternion be normalized first
def quatRotVect( q, v ):
    t = 2.0 * np.cross(q[1:4], v)
    result = v + q[0] * t + np.cross(q[1:4], t)
    return result

def quatRotVectArray( q, v ):
    t = 2.0 * np.cross(q[:,1:4], v)
    result = v + (q[:,0] * t.T).T + np.cross(q[:,1:4], t)
    return result


#  Find quaternion interpolation between two quaterions.  Blend must be 0 to 1.
#  Reference:  http://physicsforgames.blogspot.com/2010/02/quaternions.html
def quatNLerp( q1, q2, blend):
    result = np.zeros(4)

    dot = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]
    blendI = 1.0 - blend

    if dot < 0.0:
        tmpF = np.zeros(4)
        tmpF = -q2
        result = blendI*q1 + blend*tmpF
    else:
        result = blendI*q1 + blend*q2

    # Normalize quaternion
    result = result / np.linalg.norm(result)
    
    return result


#  * This will convert from quaternions to euler angles.  Ensure quaternion is previously normalized. 
#  * q(4,1) -> euler[phi;theta;psi] (rad)
#  *
#  * Reference: http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
def quat2euler( q ):
    theta = np.zeros(3)
    
    theta[0] = np.arctan2( 2 * (q[0]*q[1] + q[2]*q[3]), 1 - 2 * (q[1]*q[1] + q[2]*q[2]) )
    theta[1] = np.arcsin(  2 * (q[0]*q[2] - q[3]*q[1]) )
    theta[2] = np.arctan2( 2 * (q[0]*q[3] + q[1]*q[2]), 1 - 2 * (q[2]*q[2] + q[3]*q[3]) )
    return theta

def quat2eulerArray( q ):
    theta = np.empty((np.shape(q)[0],3))
    
    theta[:,0] = np.arctan2( 2 * (q[:,0]*q[:,1] + q[:,2]*q[:,3]), 1 - 2 * (q[:,1]*q[:,1] + q[:,2]*q[:,2]) )
    theta[:,1] = np.arcsin(  2 * (q[:,0]*q[:,2] - q[:,3]*q[:,1]) )
    theta[:,2] = np.arctan2( 2 * (q[:,0]*q[:,3] + q[:,1]*q[:,2]), 1 - 2 * (q[:,2]*q[:,2] + q[:,3]*q[:,3]) )
    return theta



#  * This will convert from euler angles to quaternion vector
#  * phi, theta, psi -> q(4,1)
#  * euler angles in radians
def euler2quat( euler ):
    q = np.zeros(4)
    hphi = euler[0] * 0.5
    hthe = euler[1] * 0.5
    hpsi = euler[2] * 0.5

    shphi = np.sin(hphi)
    chphi = np.cos(hphi)

    shthe = np.sin(hthe)
    chthe = np.cos(hthe)

    shpsi = np.sin(hpsi)
    chpsi = np.cos(hpsi)

    q[0] = chphi * chthe * chpsi + shphi * shthe * shpsi
    q[1] = shphi * chthe * chpsi - chphi * shthe * shpsi
    q[2] = chphi * shthe * chpsi + shphi * chthe * shpsi
    q[3] = chphi * chthe * shpsi - shphi * shthe * chpsi
    return q

def euler2quatArray( euler ):
    q = np.zeros((np.shape(euler)[0],4))
    hphi = euler[:,0] * 0.5
    hthe = euler[:,1] * 0.5
    hpsi = euler[:,2] * 0.5

    shphi = np.sin(hphi)
    chphi = np.cos(hphi)

    shthe = np.sin(hthe)
    chthe = np.cos(hthe)

    shpsi = np.sin(hpsi)
    chpsi = np.cos(hpsi)

    q[:,0] = chphi * chthe * chpsi + shphi * shthe * shpsi
    q[:,1] = shphi * chthe * chpsi - chphi * shthe * shpsi
    q[:,2] = chphi * shthe * chpsi + shphi * chthe * shpsi
    q[:,3] = chphi * chthe * shpsi - shphi * shthe * chpsi
    return q

# NE to heading/body frame
def psiDCM(psi):
    cpsi = cos(psi) # cos(psi)
    spsi = sin(psi) # sin(psi)
 
    DCM = r_[
           c_[  cpsi, spsi ],
           c_[ -spsi, cpsi ],
           ]
 
    return DCM  


def DCMpsi(A):
    psi = arctan2( A[0,1], A[0,0] )
    
    return psi


# NED to body frame - In the 1-2-3 (roll, pitch, yaw) order
def eulerDCM(euler):
    cphi = cos(euler[0]) # cos(phi)
    cthe = cos(euler[1]) # cos(theta)
    cpsi = cos(euler[2]) # cos(psi)
 
    sphi = sin(euler[0]) # sin(phi)
    sthe = sin(euler[1]) # sin(theta)
    spsi = sin(euler[2]) # sin(psi)
 
    DCM = r_[
           c_[  cthe*cpsi,                        cthe*spsi,                     -sthe  ],
           c_[ -cphi*spsi + sphi*sthe*cpsi,       cphi*cpsi + sphi*sthe*spsi,     sphi*cthe ],
           c_[  sphi*spsi + cphi*sthe*cpsi,      -sphi*cpsi + cphi*sthe*spsi,     cphi*cthe ],
           ]
 
    return DCM  


def DCMeuler(A):
    phi =  arctan2( A[1,2], A[2,2] )
    the =  arcsin( -A[0,2] )
    psi =  arctan2( A[0,1], A[0,0] )

    return r_[ phi, the, psi ]


def DCMeulerToPsi(A, phi, the):
    psi =  arctan2( A[0,1]/cos(the), A[0,0]/cos(the) )

    return r_[ phi, the, psi ]


# /*
#  * This will construct a direction cosine matrix from
#  * quaternions in the standard rotation sequence
#  * [phi][theta][psi] from NED to body frame
#  *
#  * body = tBL(3,3)*NED
#  * q(4,1)
#  *
#  * Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
#  */
def quatDCM(q):
    DCM = r_[
           c_[ 1.0 - 2 * (q[2]*q[2] + q[3]*q[3]),       2 * (q[1]*q[2] + q[0]*q[3]),       2 * (q[1]*q[3] - q[0]*q[2]) ],
           c_[       2 * (q[1]*q[2] - q[0]*q[3]), 1.0 - 2 * (q[1]*q[1] + q[3]*q[3]),       2 * (q[2]*q[3] + q[0]*q[1]) ],
           c_[       2 * (q[1]*q[3] + q[0]*q[2]),       2 * (q[2]*q[3] - q[0]*q[1]), 1.0 - 2 * (q[1]*q[1] + q[2]*q[2]) ],
           ]        
        
    return DCM

#  * This will construct quaternions from a direction cosine 
#  * matrix in the standard rotation sequence.
#  * [phi][theta][psi] from NED to body frame
#  *
#  * body = tBL(3,3)*NED
#  * q(4,1)
#  *
#  * Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
def DCMquat(mat):
    q = np.zeros(4)

    q[0] = 0.5 * np.sqrt(1.0 + mat[0,0] + mat[1,1] + mat[2,2])
    
    d = 1.0 / (4.0 * q[0])
    
    q[1] = d * (mat[1,2] - mat[2,1])
    q[2] = d * (mat[2,0] - mat[0,2])
    q[3] = d * (mat[0,1] - mat[1,0])

    return q


#  * This will construct the euler omega-cross matrix
#  * wx(3,3)
#  * p, q, r (rad/sec)
def eulerWx( euler ):
    mat = np.zeros((3,3))

    p = euler[0]
    q = euler[1]
    r = euler[2]

    # Row 1
    mat[0,0] =  0
    mat[0,1] = -r
    mat[0,2] =  q
    # Row 2
    mat[1,0] =  r
    mat[1,1] =  0
    mat[1,2] = -p
    # Row 3
    mat[2,0] = -q
    mat[2,1] =  p
    mat[2,2] =  0
    
    return mat


#  * This will construct the quaternion omega matrix
#  * W(4,4)
#  * p, q, r (rad/sec)
def quatW( euler ):

    mat = np.zeros((4,4))
    p = euler[0] * 0.5
    q = euler[1] * 0.5
    r = euler[2] * 0.5

    # Row 1
    mat[0,0] =  0
    mat[0,1] = -p
    mat[0,2] = -q
    mat[0,3] = -r
    # Row 2
    mat[1,0] =  p
    mat[1,1] =  0
    mat[1,2] =  r
    mat[1,3] = -q
    # Row 3
    mat[2,0] =  q
    mat[2,1] = -r
    mat[2,2] =  0
    mat[2,3] =  p
    # Row 4
    mat[3,0] =  r
    mat[3,1] =  q
    mat[3,2] = -p
    mat[3,3] =  0
    
    return mat


# *   Convert quaternion to rotation axis (and angle).  Quaternion must be normalized.
def quatRotAxis( q ):

    # Normalize quaternion
    q = q / np.linalg.norm(q)

    theta = np.arccos( q[0] ) * 2.0
    sin_a = np.sqrt( 1.0 - q[0] * q[0] )

    if np.fabs( sin_a ) < 0.0005: 
        sin_a = 1.0

    d = 1.0 / sin_a;

    pqr = np.zeros(3)
    pqr[0] = q[1] * d;
    pqr[1] = q[2] * d;
    pqr[2] = q[3] * d;
    
    print(theta * 180/pi)

    return pqr


#  *  Compute the derivative of the Euler_t angle psi with respect
#  * to the quaternion Q.  The result is a row vector
#  *
#  * d(psi)/d(q0)
#  * d(psi)/d(q1)
#  * d(psi)/d(q2)
#  * d(psi)/d(q3)
def dpsi_dq( q ):
    dq = np.zeros(4)

    t1  = 1 - 2 * (q[2]*q[2] + q[3]*q[2])
    t2  = 2 * (q[1]*q[2] + q[0]*q[3])
    err = 2 / ( t1*t1 + t2*t2 )

    dq[0] = err * (q[3]*t1)
    dq[1] = err * (q[2]*t1)
    dq[2] = err * (q[1]*t1 + 2 * q[2]*t2)
    dq[3] = err * (q[0]*t1 + 2 * q[3]*t2)

    return dq


#  Find Earth Centered Earth Fixed coordinate from LLA
#
#  lla[0] = latitude (decimal degree)
#  lla[1] = longitude (decimal degree)
#  lla[2] = msl altitude (m)
def lla2ecef( lla, metric=True):
    e = 0.08181919084262    # Earth first eccentricity: e = sqrt((R^2-b^2)/R^2);
    # double R, b, Rn, Smu, Cmu, Sl, Cl;

    # Earth equatorial and polar radii (from flattening, f = 1/298.257223563;
    if metric:
        R = 6378137         # m
        # Earth polar radius b = R * (1-f)
        b = 6356752.31424518
    else:
        R = 20925646.3255   # ft
        # Earth polar radius b = R * (1-f)
        b = 20855486.5953  

    LLA = lla * pi/180;

    Smu = sin(LLA[:,0]);
    Cmu = cos(LLA[:,0]);
    Sl  = sin(LLA[:,1]);
    Cl  = cos(LLA[:,1]);
    
    # Radius of curvature at a surface point:
    Rn = R / np.sqrt(1 - e*e*Smu*Smu)

    Pe = np.zeros(np.shape(LLA))
    Pe[:,0] = (Rn + LLA[:,2]) * Cmu * Cl
    Pe[:,1] = (Rn + LLA[:,2]) * Cmu * Sl
    Pe[:,2] = (Rn*b*b/R/R + LLA[:,2]) * Smu
    
    return Pe

#  Find NED (north, east, down) from LLAref to LLA
#
#  lla[0] = latitude (decimal degree)
#  lla[1] = longitude (decimal degree)
#  lla[2] = msl altitude (m)
def lla2ned( llaRef, lla ):
    a2d = 111120.0      # = DEG2RAD * earth_radius_in_meters

    deltaLLA = np.zeros(np.shape(lla))    
    deltaLLA[:,0] = lla[:,0] - llaRef[0]
    deltaLLA[:,1] = lla[:,1] - llaRef[1]
    deltaLLA[:,2] = lla[:,2] - llaRef[2]
    
    ned = np.zeros(np.shape(lla))
    ned[:,0] =  deltaLLA[:,0] * a2d
    ned[:,1] =  deltaLLA[:,1] * a2d * cos(llaRef[0] * pi/180)
    ned[:,2] = -deltaLLA[:,2]

    return ned


#  Find NED (north, east, down) from LLAref to LLA
#
#  lla[0] = latitude (decimal degree)
#  lla[1] = longitude (decimal degree)
#  lla[2] = msl altitude (m)
def lla2ned_single( llaRef, lla ):
    a2d = 111120.0      # = DEG2RAD * earth_radius_in_meters

    deltaLLA = np.zeros(np.shape(lla))    
    deltaLLA[0] = lla[0] - llaRef[0]
    deltaLLA[1] = lla[1] - llaRef[1]
    deltaLLA[2] = lla[2] - llaRef[2]
    
    ned = np.zeros(np.shape(lla))
    ned[0] =  deltaLLA[0] * a2d
    ned[1] =  deltaLLA[1] * a2d * cos(llaRef[0] * pi/180)
    ned[2] = -deltaLLA[2]

    return ned


#  Find Delta LLA of NED (north, east, down) from LLAref
#
#  lla[0] = latitude (decimal degree)
#  lla[1] = longitude (decimal degree)
#  lla[2] = msl altitude (m)
def ned2DeltaLla( ned, llaRef ):
    invA2d = 1.0 / 111120.0      # 1 / radius

    deltaLLA = np.zeros(np.shape(ned))
    deltaLLA[:,0] =  ned[:,0] * invA2d
    deltaLLA[:,1] =  ned[:,1] * invA2d / cos(llaRef[0] * pi/180)
    deltaLLA[:,2] = -ned[:,2]

    return deltaLLA

#  Find LLA of NED (north, east, down) from LLAref
#
#  lla[0] = latitude (decimal degree)
#  lla[1] = longitude (decimal degree)
#  lla[2] = msl altitude (m)
def ned2lla( ned, llaRef ):
    deltaLLA = ned2DeltaLla( ned, llaRef )
    
    lla = np.zeros(np.shape(ned))
    lla[:,0] = llaRef[:,0] + deltaLLA[:,0]
    lla[:,1] = llaRef[:,1] + deltaLLA[:,1]
    lla[:,2] = llaRef[:,2] + deltaLLA[:,2]

    return lla

def rotate_ned2ecef( latlon ):
    R = np.zeros((3,3))
    # Smu, Cmu, Sl, Cl

    Smu = sin(latlon[0])
    Cmu = cos(latlon[0])
    Sl = sin(latlon[1])
    Cl = cos(latlon[1])

    R[0,0] = -Smu * Cl
    R[0,1] = -Sl
    R[0,2] = -Cmu * Cl
    R[1,0] = -Smu * Sl
    R[1,1] = Cl
    R[1,2] = -Cmu * Sl
    R[2,0] = Cmu
    R[2,1] = 0.0
    R[2,2] = -Smu
    return R

def rotate_ecef2ned( latlon ):
    R = np.zeros((3,3))
    # Smu, Cmu, Sl, Cl

    Smu = sin(latlon[0])
    Cmu = cos(latlon[0])
    Sl = sin(latlon[1])
    Cl = cos(latlon[1])

    R[0,0] = -Smu * Cl
    R[0,1] = -Smu * Sl
    R[0,2] = Cmu

    R[1,0] = -Sl
    R[1,1] = Cl
    R[1,2] = 0.0

    R[2,0] = -Cmu * Cl
    R[2,1] = -Cmu * Sl
    R[2,2] = -Smu
    return R


#  * NED to Euler_t
def nedEuler(ned):
    euler = np.zeros(np.shape(ned))
    euler[:,2] = arctan2( ned[:,1], ned[:,0] )
    euler[:,1] = arctan2( -ned[:,2], np.sqrt( ned[:,0]**2 + ned[:,1]**2 ) )
    euler[:,0] = 0
    return euler


#  * Euler_t to NED
def eulerNed( e ):
    ned = np.zeros(3)
    v = r_[ 1., 0., 0.]
    vectorRotateBodyToInertial( v, e, ned )
    return ned


# Rotate theta eulers from body to inertial frame by ins eulers, in order: phi, theta, psi
def eulerRotateBodyToInertial2( e, rot ):
    eResult = np.zeros(np.shape(e))

    Ai = eulerDCM(rot)                  # use estimate

    for i in range(0,np.shape(eResult)[0]):
        # Create DCMs (rotation matrices)
        At = eulerDCM(e[i,:])
         
        # Apply INS Rotation to Desired Target vector
        AiAt    = dot(Ai, At)               # Apply rotation
        eResult[i,:] = DCMeuler(AiAt)            # Pull out new eulers

    return eResult

# Rotate theta eulers from inertial to body frame by ins eulers, in order: psi, theta, phi
def eulerRotateInertialToBody2( e, rot ):
    eResult = np.zeros(np.shape(e))

    Ai = eulerDCM(rot)                  # use estimate

    for i in range(0,np.shape(eResult)[0]):
        # Create DCMs (rotation matrices)
        At = eulerDCM(e[i,:])
         
        # Apply INS Rotation to Desired Target vector
        AiAt    = dot(Ai.T, At)             # Apply rotation
        eResult[i,:] = DCMeuler(AiAt)            # Pull out new eulers

    return eResult

# Rotate vector from inertial to body frame by euler angles, in order: psi, theta, phi
def vectorRotateInertialToBody( vIn, eRot ):
    vResult = np.zeros(np.shape(vIn))

    for i in range(0,np.shape(vResult)[0]):
        # Create DCM (rotation matrix)
        DCM = eulerDCM(eRot[i,:])
        # Apply rotation to vector
        vResult[i,:] = np.dot( DCM, vIn[i,:] )
    
    return vResult

# Rotate vector from body to inertial frame by euler angles, in order: phi, theta, psi
def vectorRotateBodyToInertial( vIn, eRot ):
    vResult = np.zeros(np.shape(vIn))

    for i in range(0,np.shape(vResult)[0]):
        # Create DCM (rotation matrix)
        DCM = eulerDCM(eRot[i,:])
        # Apply rotation to vector
        vResult[i,:] = np.dot( DCM.T, vIn[i,:] )
    
    return vResult

# Rotate vector from inertial to body frame by euler angles, in order: psi, theta, phi
def vectorRotateInertialToBody2( vIn, eRot ):
    vResult = np.zeros(np.shape(vIn))

    # Create DCM (rotation matrix)
    DCM = eulerDCM(eRot)

    for i in range(0,np.shape(vResult)[0]):
        # Apply rotation to vector
        vResult[i,:] = np.dot( DCM, vIn[i,:] )
    
    return vResult

# Rotate vector from body to inertial frame by euler angles, in order: phi, theta, psi
def vectorRotateBodyToInertial2( vIn, eRot ):
    vResult = np.zeros(np.shape(vIn))

    # Create DCM (rotation matrix)
    DCM = eulerDCM(eRot)

    for i in range(0,np.shape(vResult)[0]):
        # Apply rotation to vector
        vResult[i,:] = np.dot( DCM.T, vIn[i,:] )
    
    return vResult


# Find euler angles of a vector (no roll)
def vectorEuler( v ):
    psi     = np.arctan2( v[1], v[0] )
    dcm     = psiDCM( psi )
    v2      = np.dot( dcm, v[0:2] )
    theta   = np.arctan2( -v[2], v2[0] )
    result  = r_[ 0., theta, psi ]
#     print("v:", v, " v2:", v2, " e:", result * 180/pi)
    return result


def vectorQuat( v ):
    return euler2quat( vectorEuler(v) )


def unwrapAngle(angle):
    
    twoPi = pi*2
    
    for i in range(0,np.shape(angle)[0]):
        while angle[i] < -pi:
            angle[i] += twoPi
        while angle[i] >  pi:
            angle[i] -= twoPi
            
    return angle

# Non-accelerated gravity used to determine attitude
def accellToEuler(acc):
    euler = np.zeros(np.shape(acc))
    
    euler[:,0] = np.arctan2( -acc[:,1], -acc[:,2] )
    euler[:,1] = np.arctan2(  acc[:,0], np.sqrt( acc[:,1]*acc[:,1] + acc[:,2]*acc[:,2]) );
    return euler;

def acc2AttAndBias(acc):
    att = np.zeros(np.shape(acc))
    bias = np.zeros(np.shape(acc))
   
#     pe = pt.cPlot()
       
    for i in range(0,4):
        att = accellToEuler(acc-bias)
    
        gIF = np.r_[ 0, 0, -9.80665 ]
        for i in range(0,np.shape(bias)[0]):
            # Create DCM (rotation matrix)
            DCM = eulerDCM(att[i,:])
            # Apply rotation to vector: inertial -> body
            g = np.dot( DCM, gIF )
            bias[i,:] = acc[i,:] - g
            
#         pe.plot3Axes(1, range(0,np.shape(bias)[0]), bias, 'Bias', 'm/s^2')
         
    return [att, bias]

def norm(v, axis=None):
    return np.sqrt(np.sum(v*v, axis=axis))

qmat_matrix = np.array([[[1.0, 0, 0, 0],
                         [0, -1.0, 0, 0],
                         [0, 0, -1.0, 0],
                         [0, 0, 0, -1.0]],
                        [[0, 1.0, 0, 0],
                         [1.0, 0, 0, 0],
                         [0, 0, 0, 1.0],
                         [0, 0, -1.0, 0]],
                        [[0, 0, 1.0, 0],
                         [0, 0, 0, -1.0],
                         [1.0, 0, 0, 0],
                         [0, 1.0, 0, 0]],
                        [[0, 0, 0, 1.0],
                         [0, 0, 1.0, 0],
                         [0, -1.0, 0, 0],
                         [1.0, 0, 0, 0]]])

def qmult(q1, q2):
    if q1.shape[0] == 1 and q2.shape[0] == 1:
        dots = np.empty_like(q2)
        for i in range(q2.shape[0]):
            dots[i, :] = qmat_matrix.dot(q2.T).squeeze().dot(q1.T).T
    elif q1.shape[0] == 1 and q2.shape[0] != 1:
        dots = np.empty_like(q2)
        for i in range(q2.shape[0]):
            dots[i, :] = qmat_matrix.dot(q2[i, :].T).squeeze().dot(q1.T).T
    elif q1.shape[0] != 1 and q2.shape[0] == 1:
        dots = np.empty_like(q2)
        for i in range(q2.shape[0]):
            dots[i, :] = qmat_matrix.dot(q2.T).squeeze().dot(q1[i,:].T).T
    elif q1.shape[0] == q2.shape[0]:
        dots = np.empty_like(q2)
        for i in range(q2.shape[0]):
            dots[i, :] = qmat_matrix.dot(q2[i,:].T).squeeze().dot(q1[i, :].T).T
    else:
        raise Exception("Incompatible quaternion arrays -- cannot multiply")

    # print qmat_matrix.dot(q2.T).squeeze()
    # print np.tensordot(qmat_matrix.T, q2, axes=[0,1]).T.squeeze()
    # dots = np.empty((2,4,4))
    # for i in range(q1.shape[0]):
    #      dots[i,:] = qmat_matrix.dot(q2[i,:].T)
    # tensordots = np.tensordot(qmat_matrix.T, q2, axes=[0,1]).T
    # dots = np.empty_like(q1)
    # for i in range(q1.shape[0]):
    #      dots[i,:] = qmat_matrix.dot(q2[i,:].T).squeeze().dot(q1[i,:].T).T
    # tensordots = np.tensordot(q1, np.tensordot(qmat_matrix.T, q2, axes=[0,1]), axes=1).T
    # print "dots = ", dots, "\ntensordots = ", tensordots
    # print "diff = ", dots - tensordots

    # return np.tensordot(q1, np.tensordot(q2.T, qmat_matrix.T, axes=[0,1]).T, axes=1)[0].T
    return dots

def qlog(q):
    q *= np.sign(q[:,0])[:,None]
    norm_v = norm(q[:,1:], axis=1)
    out = np.empty((len(q), 3))
    idx = norm_v > 1e-4
    out[~idx] = 2.0 * np.sign(q[~idx, 0, None]) * q[~idx,1:]
    out[idx] = (2.0 * np.arctan2(norm_v[idx,None], q[idx,0,None])) * q[idx,1:]/norm_v[idx,None]
    return out

def qexp(v):
    out = np.empty((len(v), 4))
    norm_v = norm(v, axis=1)
    idx = norm_v > 1e-4
    out[~idx, 0]  =  1
    out[~idx, 1:] = v[~idx,:]
    out[idx] = np.cos(norm_v[idx,None]/2.0)
    out[idx, 1:] = np.sin(norm_v[idx,None]/2.0) * v[idx,:]/norm_v[idx,None]
    return out

def qinv(q):
    qcopy = q.copy()
    qcopy[:,1:] *= -1.0
    return qcopy

def qboxplus(q, v):
    return qmult(q, qexp(v))

def qboxminus(q1, q2):
    return qlog(qmult(qinv(q2), q1))

# Implementation of "Mean of Sigma Points" from Integrating Generic Sensor Fusion Algorithms with
# Sound State Representations through Encapsulation of Manifolds by Hertzberg et. al.
# https://arxiv.org/pdf/1107.1119.pdf p.13
def meanOfQuat(q):
    n = float(q.shape[0])
    mu = q[None,0,:]
    prev_mu = None
    iter = 0
    while prev_mu is None or norm(qboxminus(mu, prev_mu)) > 1e-3:
        iter +=1
        prev_mu = mu
        mu = qboxplus(mu, np.sum(qboxminus(q, mu), axis=0)[None,:]/n)
    assert np.abs(1.0 - norm(mu)) <= 1e03
    return mu

def meanOfQuatArray(q):
    assert q.shape[2] == 4
    mu = np.empty((q.shape[0], 4))
    for i in tqdm(range(q.shape[0])):
        mu[None,i,:] = meanOfQuat(q[i,:,:])
    return mu


# def salemUtLla():
#     return np.r_[ 40.0557114, -111.6585476, 1426.77 ]    # // (deg,deg,m) Lat,Lon,Height above sea level (not HAE, height above ellipsoid)
# 
# def salemUtMagDecInc():
#     return np.r_[ 0.20303997, 1.141736219 ]    # (rad) Declination: 11.6333333 deg or 11 deg 38', Inclination: -65.4166667 deg or 65 deg 25'

if __name__ == '__main__':
    q = np.random.random((5000, 4))
    q = q / norm(q, axis=1)[:,None]
    # Find the mean Quaternion

    q1 = q[0:2, :]
    q2 = q[2:4, :]

    q3 = qmult(q1, q2)

    print("q2 =", q2)
    print("math =", qmult(qinv(q1), q3))
    assert np.sqrt(np.sum(np.square(qmult(qinv(q1), q3) - q2))) < 1e-8

    mu = meanOfQuat(q)

    qarr = np.random.random((5000, 25, 4))
    qarr = qarr * 1.0/norm(qarr, axis=2)[:,:,None]
    mu = meanOfQuatArray(qarr)
    print(mu)

    pass