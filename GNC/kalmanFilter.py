import numpy as np
from GNC.controlConstants import *

def kalman_filter(x, u, Z, A, B, dt, engine_length, P):
    
    # Calculate Process Noise Matrix
    v = np.array([0.5*dt*dt, 0.5*dt*dt, 0.5*dt*dt, dt, dt, dt, 0.5*dt*dt, 0.5*dt*dt, 0, dt, dt, 0]).T
    var = SIGMA_PROCESS_NOISE
    Q = np.outer(v * var, v.T)

    # Initialization
    F = np.identity(len(x)) + (A * dt) + (((A * dt) ** 2) / 2) + (((A * dt) ** 3) / 6) 
    H = np.eye(12)
    R = np.eye(12) * 1000
    
    # Prediction step
    x_next, P_next = predict_step(x, u, F, B, P, Q, dt)
    x_fit, P_fit = update_step(Z, H, R, x_next, P_next)

    return x_fit, P_fit
        
def predict_step(x, u, F, B, P_prev, Q, dt):
    x_next = np.dot(F, x) # Predicted State Estimate
    P_next = np.dot(np.dot(F, P_prev), F.T) + Q # Predicted Estimate Covariance
    return x_next, P_next

def update_step(z, H, R, x_next, P_next):
    y = z - np.dot(H, x_next) # Measurement pre-fit residual
    S = np.dot(np.dot(H, P_next), H.T) + R # Covariance pre-fit residual
    K = np.dot(np.dot(P_next, H.T), np.linalg.inv(S)) # Optimal Kalman Gain
    x_fit = x_next #+ np.dot(K, y) # Updated state estimate
    P_fit = np.dot((np.identity(len(P_next)) - np.dot(K, H)), P_next) # Updated covariance
    return x_fit, P_fit
    
    