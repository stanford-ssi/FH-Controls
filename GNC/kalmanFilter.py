import numpy as np
import math
from GNC.controlConstants import *

def kalman_filter(x, u, Z, A, B, dt, P):
    """ Kalman filter"""
    # Calculate Process Noise Matrix
    Q = np.eye(12)

    # Initialization
    F = np.array([
        [1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    ])
    # H assumes sensor format [x y z xdot ydot zdot z xdot ydot zdot p y r pdot ydot rdot]
    H = np.array([
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],       
    ])
  
    R = np.eye(16)
    
    # Prediction step
    x_next, P_next = predict_step(x, u, F, B, P, Q, dt)
    x_fit, P_fit = update_step(Z, H, R, x_next, P_next)

    return x_fit, P_fit
        
def predict_step(x, u, F, B, P_prev, Q, dt):
    """ Prediction step for kalman filter"""
    x_next = np.dot(F, x) # Predicted State Estimate
    P_next = np.dot(np.dot(F, P_prev), F.T) + Q # Predicted Estimate Covariance
    return x_next, P_next

def update_step(z, H, R, x_next, P_next):
    """ Update step for kalman filter"""
    prediction = np.dot(H, x_next)
    for i in range(len(z)):
        if math.isnan(z[i]):
            z[i] = prediction[i]
     
    y = z - prediction # Measurement pre-fit residual
    S = np.dot(np.dot(H, P_next), H.T) + R # Covariance pre-fit residual
    K = np.dot(np.dot(P_next, H.T), np.linalg.inv(S)) # Optimal Kalman Gain
    x_fit = x_next + np.dot(K, y) # Updated state estimate
    P_fit = np.dot((np.identity(len(P_next)) - np.dot(K, H)), P_next) # Updated covariance
    return x_fit, P_fit
    
    