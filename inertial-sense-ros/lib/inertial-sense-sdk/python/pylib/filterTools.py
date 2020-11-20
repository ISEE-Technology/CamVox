'''
Created on Feb 26, 2014

@author: waltj
'''
import numpy as np
from numpy import r_, c_, dot
from pickle import FALSE

def integral(time, data):
    size = np.shape(time)[0]
    result = np.zeros(np.shape(data))
    integral = data[0]
    dt = 0
    for i in range(1, size):
        dt = time[i] - time[i-1]
        integral += data[i] * dt
        result[i] = integral
    return result

# Non-causal, depends on future estimates and output is not delayed.
def derivative(time, data, delta=1):
    size = np.shape(time)[0]
    result = np.zeros(np.shape(data))
    for i in range(delta, size-delta):
        dt = time[i+delta] - time[i-delta]
        if dt != 0:
            result[i] = (data[i+delta] - data[i-delta]) / dt
        else:
            result[i] = 0;
            
    return result

# Causal, doesn't depend on future value, but output is 1/2 a sample delayed.
def derivativeCausal(time, data, delta=1):
    size = np.shape(time)[0]
    result = np.zeros(np.shape(data))
    for i in range(delta, size):
        dt = time[i] - time[i-delta]
        if dt != 0:
            result[i] = (data[i] - data[i-delta])/dt
        else:
            result[i] = 0;
            
    return result
    
# delta = number of adjacent points to average in        
def meanDerivative(time, data, derDelta=1, avgdelta=1):
    der = derivative(time, data, derDelta)
            
    return smooth(der, avgdelta)


# Smooth data with delta number of adjacent samples (i.e. delta=2 averages 
# the 2 samples before and after with the current sample) 
def smooth(data, delta):
    size = np.shape(data)[0]
    result = np.copy(data)
    mul = 1/(delta*2.0 + 1.0)
    for i in range(delta, size-delta):
        result[i] = data[i]
        for j in range(1,delta+1):
            result[i] += data[i-j] + data[i+j]
        result[i] *= mul

    return result


def smooth2(data, delta, step):
    size = np.shape(data)[0]
    size2 = int(size/step)

    result = data[:size2]
    mul = 1/(delta*2.0 + 1.0)
    for i in range(delta, size-delta):
        k = i/step
        if k < size2:
            result[k] = data[i]
            if i+delta < size:
                for j in range(1,delta+1):
                        result[k] += data[i-j] + data[i+j]
                result[k] *= mul

    result[-1] = result[-2]

    return result


def lpf(data, cornerFreqHz, dt=None, time=None, initVal=None):
    size = np.shape(data)[0]
    
    if dt is None and time is None:
        print("Error: both dt and time are None")
        return
    
    # Find average dt in time
    if time is None and dt is None:
        dt = np.mean( time[1:] - time[0:-1] )
#         print "LPF mean dt: ", dt
    
    result  = np.copy(data)
    alpha   = dt*cornerFreqHz / (1.0 + dt*cornerFreqHz)
    beta    = 1.0 - alpha
    
    if initVal is not None:
        result[0] = initVal
    
    for i in range(1,size):
        result[i] = beta*result[i-1] + alpha*data[i]
        
    return result


def lpfNoDelay(data, cornerFreqHz, dt=None, time=None, initVal=None):
    size = np.shape(data)[0]
    
    if dt is None and time is None:
        print("Error: both dt and time are None")
        return
    
    # Find average dt in time
    if time is not None and dt is None:
        dt = np.mean( time[1:] - time[0:-1] )
#         print "LPF mean dt: ", dt
    
    result  = np.copy(data)
    result2 = np.copy(data)
    alph    = dt*cornerFreqHz / (1.0 + dt*cornerFreqHz)
    beta    = 1.0 - alph
    
    if initVal is not None:
        result[0,:] = initVal
    else:
        result[0,:] = np.mean(result, axis=0)
    
    for i in range(1,size):
        result[i,:] = beta*result[i-1,:] + alph*data[i,:]

    # init using last value
    result2[-1,:] = result[-1,:]

    # Now LPF in reverse direction
    for i in range(size-1,0,-1):
        result2[i-1,:] = beta*result2[i,:] + alph*result[i-1,:]
        
    return result2
        
        
def o1lpf(data, cornerFreqHz, dt=None, time=None, initVal=None):
    size = np.shape(data)[0]
    
    if dt is None and time is None:
        print("Error: both dt and time are None")
        return

    # Find average dt in time
    if time is not None and dt is None:
        dt = np.mean( time[1:] - time[0:-1] )
#         print "LPF mean dt: ", dt
         
    if time is None:
        time = np.arange(0, dt*size, dt)

    alph   = dt*cornerFreqHz / (1 + dt*cornerFreqHz)
    beta    = 1.0 - alph

    result  = np.copy(data)
    
    # Estimate inital model
    c1 = (data[1] - data[0])/(time[1] - time[0])
    
    for i in range(1,size):
        h = i-1

        dt = time[i] - time[h]
        
#         # Estimate next model
# #         dy = data[i] - data[h]
#         dy = data[i] - result[h]
#         d1 = dy/dt
#         
#         # Alpha filter coefficient
#         c1 = beta*c1 + alph*d1
# 
#         # Current state estimate
#         result[i] = result[h] + c1*dt
#         
#         # LPF input into current state estimate
#         result[i] = beta*result[i] + alph*data[i] 

        # Estimate next model coefficient
        # LPF filter this coefficient
        c1 = beta*c1 + alph*( (data[i] - result[h])/dt )

        # Current state estimate
        # LPF input into current state estimate
        result[i] = beta*(result[h] + c1*dt) + alph*data[i] 

    return result

# Recursive Least Squares - Linear, fixed time interval model 
# k is forgetting factor, smaller - more remembering
def rls_array(meas, k):
    s = np.shape(meas)
    if np.shape(s)[0] == 1:
        wide = 1
        meas = r_[c_[meas]]
    else:
        wide = np.shape(meas)[1]
        
    size = np.shape(meas)[0]
    est  = np.zeros(np.shape(meas))

#     print np.shape(meas), " ", meas
#     print "size: ", size
#     print "wide: ", wide 

    # Initialization
    est[0] = meas[0]
    A = r_[ c_[ np.ones(wide), np.zeros(wide) ] ]     # [ [1], [slope] ]
    Z = r_[ c_[ meas[0,:], np.ones(wide)] ].T

#     print "A: ", A
#     print "Z: ", Z

    for i in range(0, size-1):
        # Update model
        err     = meas[i] - est[i]
        X       = est[i] + k*err
        Z[0,:]  = est[i]
        A       = dot( X, dot( Z.T, np.linalg.inv( dot(Z,Z.T) ) ) )

#         A[0] = 1
#         print A
            
        # Next estimate
#         print dot( A, Z ).T
        est[i+1,:] = dot( A, Z ).T
    
    return (est, A)
    

        
        
        