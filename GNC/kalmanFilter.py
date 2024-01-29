import numpy as np

def kalman_filter(x, u, A, B, sensor_readings):
    x_next, P_next = predict_step(x, u, A, B, P_prev, Q)
    for value in sensor_readings:
        x_next, P_next = update_step(value, H, R, x_next, P_next)
    return x_next
        
def predict_step(x, u, A, B, P_prev, Q):
    x_next = (A * x) + (B * u) # Predicted State Estimate
    P_next = (A * P_prev * A.T) + Q # Predicted Estimate Covariance
    return x_next, P_next

def update_step(z, H, R, x_next, P_next):
    y = z - (H * x_next) # Measurement pre-fit residual
    S = (H * P_next * H.T) + R # Covariance pre-fit residual
    K = P_next * H.T * np.linalg.inv(S) # Optimal Kalman Gain
    x_fit = x_next + (K * y) # Updated state estimate
    P_fit = (np.identity(len(P_next)) - (K * H)) * P_next # Updated covariance
    return x_fit, P_fit
    
    