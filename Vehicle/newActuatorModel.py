import math
import numpy as np
import scipy.signal as signal
import matplotlib.pyplot as plt

class LinearActuator:
    def __init__(self, tf, ts):
        # Source: Feiler2003 
        K = 1
        T = 0.5

        # Create Transfer function
        n = [K]
        d = [T, 1]
        self.sys = signal.TransferFunction(n, d)

        self.X  = np.array([[0.0]]) # Initaial Condistions
        self.T  = np.linspace(0, int(tf/ts)*ts, num=int(tf/ts)+1)
        self.U  = np.array([[0.0]])   

    def get_output(self, u_current, t):
        if not t == 0:
            self.U = np.append(self.U, u_current)
        tout, y, x = signal.lsim(self.sys, self.U, self.T[:min(enumerate(self.T), key=lambda x: abs(x[1]-t))[0] + 1], self.X)
        
        # x and y contains 2 simulation points the newer one is the correct one.
        if not tout[-1] == 0:
            self.X = x[-1] # update state      
            return y[-1] #
        else:
            return self.X[0][0] 

# def plot_time_series(time_vector, output_vector, clear_plot=False):
#     """
#     Plots a time series given a time vector and an output vector.

#     Parameters:
#         time_vector (list or numpy array): Time vector.
#         output_vector (list or numpy array): Output vector.
#         clear_plot (bool): Whether to clear the existing plot before plotting new data.
#     """
#     if clear_plot:
#         plt.clf()

#     plt.plot(time_vector, output_vector)
#     plt.xlabel('Time')
#     plt.ylabel('Output')
#     plt.title('Time Series Plot')
#     plt.grid(True)

# A = LinearActuator()
# time = A.T
# for i in range(len(time)):
#     u_current = np.sin(time[i])
#     y = A.get_output(u_current, i)

# plot_time_series(time, y)
# plt.show()    

     