#Everything ran and it seems to be fine?
import numpy as np
import matplotlib.pyplot as plt

class PlannedTrajectory:
    def __init__(self, max_altitude, tf, ts):
        self.max_altitude = max_altitude
        self.time = np.linspace(0, int(tf/ts)*ts, num=int(tf/ts)+1) 
        self.ts = ts
        self.tf = tf

        self.ad_timing_proportion = 2 # Amount of ascent phase vs decent phase
        self.landing_timing_proportion = 2 # When in descent phase should we start the landing sequence? Higher means later
        self.ascent_duration = self.time[-1] / self.ad_timing_proportion
        self.decent_duration = tf - self.ascent_duration
        self.trajectory = self.xyz_trajectory()

    def sin_trajectory(self, t):
        return (self.max_altitude * (np.sin((2 * t * np.pi / self.tf) - (np.pi / 2)) + 1) / 2) + 0.1
        
    def xyz_trajectory(self):
        trajectory = [[0, 0, self.sin_trajectory(t)] for t in self.time]
        velocity = [[(trajectory[i+1][j] - trajectory[i][j]) / self.ts for j in range(len(trajectory[0]))] for i in range(len(trajectory)-1)]
        velocity.insert(0, [0,0,0])
        result = np.concatenate((trajectory, velocity), axis=1)

        return np.array(result)
    
    def plot_trajectory(self):
        plt.plot(self.time, self.trajectory[:,0:3], label='Variable vs Time')
        plt.plot(self.time, self.trajectory[:,4:6], label='Variable vs Time')
        plt.legend(("X Position", "Y Position", "Z Position", "X Velocity", "Y Velocity", "Z Velocity"))
        plt.xlabel('Time')
        plt.ylabel('Variable')
        plt.title('Planned Trajectory')
        plt.grid(True)
        plt.show()