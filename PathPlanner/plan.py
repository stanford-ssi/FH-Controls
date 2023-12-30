#Everything ran and it seems to be fine?
import numpy as np
import matplotlib.pyplot as plt

class PlannedTrajectory:
    def __init__(self, max_altitude, tf):
        self.max_altitude = max_altitude
        self.time = np.linspace(0, int(tf/0.1)*0.1, num=int(tf/0.1)+1) 

        self.ad_timing_proportion = 2 # Amount of ascent phase vs decent phase
        self.landing_timing_proportion = 6 # When in descent phase should we start the landing sequence? Higher means later
        self.ascent_duration = self.time[-1] / self.ad_timing_proportion
        self.decent_duration = tf - self.ascent_duration
        self.trajectory = self.xyz_trajectory()

    def z_piecewise_cubic(self, t):
        if 0 <= t <= 2.5:
            return 1.6 * (t ** 3)
        elif 2.5 <= t <= 5:
            return self.max_altitude + 1.6 * ((t - 5) ** 3)
        elif 5 <= t <= 7.5:
            return self.max_altitude - 1.6 * ((t - 5) ** 3)
        elif 7.5 <= t <= 10:
            return -1.6 * ((t - 10) ** 3) 
        else: 
            return 0

    def better(self, t):

        if t <= self.ascent_duration:
            return self.max_altitude + self.max_altitude * ((t - self.ascent_duration) / self.ascent_duration)**3
        elif t <= self.ascent_duration + ((self.landing_timing_proportion - 1) * self.decent_duration / self.landing_timing_proportion):
            return self.max_altitude - self.max_altitude * ((t - self.ascent_duration) / (self.decent_duration))**3
        else:
            h = (self.max_altitude - self.max_altitude * (((self.ascent_duration + ((self.landing_timing_proportion - 1) * self.decent_duration / self.landing_timing_proportion)) - self.ascent_duration) / (self.decent_duration))**3)
            b = (self.ascent_duration + self.decent_duration)
            a = h / ((b - (self.ascent_duration + ((self.landing_timing_proportion - 1) * self.decent_duration / self.landing_timing_proportion)))**3 - (b - (self.ascent_duration + self.decent_duration))**3)
            return a * (b - t)**3
            #return ((self.ad_timing_proportion ** 4) * self.max_altitude) * (((self.ascent_duration + self.decent_duration) - t) / (self.decent_duration))**3

    def xyz_trajectory(self):
        trajectory = [[0, 0, self.better(t)] for t in self.time]
        

        return np.array(trajectory)
    
    def plot_trajectory(self):
        plt.plot(self.time, self.trajectory, label='Variable vs Time')
        plt.xlabel('Time')
        plt.ylabel('Variable')
        plt.title('Planned Height vs Time Plot')
        plt.grid(True)
        plt.show()