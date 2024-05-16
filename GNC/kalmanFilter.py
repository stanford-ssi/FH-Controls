import numpy as np
import math
from GNC.controlConstants import *

def kalman_filter(x, u, y, A, B, dt, P):
    """ Kalman filter"""
    # Calculate Process Noise Matrix
    Q = np.eye(12)

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
    
    # Time Step
    x_next, P_next = predict_step(x, u, A, B, P, Q, dt)
    
    # Measurement Step
    # z = np.dot(H, x_next) # Expected Measurement based on state
    # x_fit, P_fit = update_step(x_next, y, z, H, R, P_next)

    return x_next, P_next
        
def predict_step(x, u, A, B, P_prev, Q, dt):
    """ Prediction step for kalman filter"""
    A_new = np.eye(12) + A*dt
    B_new = B*dt
    print(A_new[9])
    x_next = A_new @ x + B_new @ u  #Predicted State Estimate
    P_next = (A_new @ P_prev @ A_new.T) + Q # Predicted Estimate Covariance
    return x_next, P_next

def update_step(x_next, y, z, H, R, P_next):
    """ Update step for kalman filter"""
    
    # Clean if there are no measurements from a sensor
    for i in range(len(y)):
        if math.isnan(y[i]):
            y[i] = z[i]
    
    S = (H @ P_next @ H.T) + R
    K = P_next @ H.T @ np.linalg.inv(S)
    x_fit = x_next + np.dot(K, (y-z))
    
    chunk = np.eye(12) - (K @ H)
    P_fit = (chunk @ P_next @ chunk.T) + (K @ R @ K.T)
    
    return x_fit, P_fit
    
    