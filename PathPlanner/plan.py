#This will be better soon
import numpy as np

# Time (t start = 0 seconds, t finish = 10 seconds, dt = 0.1 * 100 = 10 seconds (tf - ti))
time = np.linspace(0, 10, 100)

#Four-Piece Function for 
def z_piecewise_cubic(t):
    if t <= 2.5:
        return 1.6 * (t ** 3)
    elif t <= 5:
        return 50 + 1.6 * ((t - 5) ** 3)
    elif t <= 7.5:
        return 50 - 1.6 * ((t - 5) ** 3)
    else:
        return -1.6 * ((t - 10) ** 3)

#For Plotting
trajectory = np.array([0, 0, piecewise_cubic(t) for t in time]) 
