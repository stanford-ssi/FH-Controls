import math
import numpy as np
import scipy.signal as signal
import matplotlib.pyplot as plt

class LinearActuator:
    def __init__(self, tf, ts):
        # Source: Feiler2003 
        K = 1
        T = 0.1

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

########################### Uncomment code below this line and run this file to tune actuator model  ###############################

# tf = 2
# ts = 0.01
# A = LinearActuator(tf, ts)
# time = A.T
# y = []
# u = []
# for i in range(len(time)):
#     u_current = 1#np.sin(2 * time[i]) # Set this line to be the signal you want to track ie step function, sin wave, ect
#     u.append(u_current)
#     y.append(A.get_output(u_current, i*ts))
# plt.plot(time, u)
# plt.plot(time, y)
# plt.xlabel('Time')
# plt.ylabel('Output')
# plt.title('Time Series Plot')
# plt.grid(True)
# plt.show()    

     