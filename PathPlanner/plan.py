import numpy as np
from scipy.optimize import linprog
import matplotlib.pyplot as plt
import Graphing.plotter

class PlannedTrajectory:
    def __init__(self, max_altitude, time, target, dt):
        self.max_altitude = max_altitude
        self.target = target
        self.dt = dt
        self.t_total = time
        self.trajectory = self.cubic_trajectory()
    def plan_path(self):  
        
    
