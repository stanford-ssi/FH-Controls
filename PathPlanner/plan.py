#Everything ran and it seems to be fine?
import numpy as np

class PlannedTrajectory:
    def __init__(self, max_altitude):
        self.max_altitude = max_altitude
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

    def xyz_trajectory(self):
        time = np.linspace(0, 10, 100)
        trajectory = [[0, 0, self.z_piecewise_cubic(t)] for t in time]
        return np.array(trajectory)

