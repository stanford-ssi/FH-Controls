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

    def cubic_trajectory(self):
        # Define time intervals
        time_intervals = np.linspace(0, int(self.t_total/self.dt)*self.dt, num=int(self.t_total/self.dt)+1)

        # Determine the acceleration period (first half of time)
        acceleration_time = self.t_total / 2
        acceleration = 1.5 * self.max_altitude / acceleration_time ** 3

        # Calculate the trajectory based on the cubic function
        trajectory = [
            [0,0,(8 * acceleration) * (t)] if t <= acceleration_time else
            [0,0,(self.max_altitude - (acceleration / 3) * (self.t_total - t) ** 3)] for t in time_intervals
        ]
        #Graphing.plotter.plot_variable_vs_time(np.array(trajectory)[:,2], self.dt, self.t_total, name="Planned Trajectory")
        return np.array(trajectory)


    def plan_path(self):
        """ Plan trajectory. For now, trajectory starts straight up, hoovers, and then decends towards target"""

        # Define Time
        t = np.linspace(0, int(self.t_total/self.dt)*self.dt, num=int(self.t_total/self.dt))

        # Write Vertical Portion
        vertical_states = self.vertical_portion()
        hoover_states = self.hoover_portion()
        #descent_states = self.descent_portion()
        return(np.concatenate((vertical_states, hoover_states), axis=0))

    def vertical_portion(self):
        # For now, simple linear ascent profile
        final_time = int(self.ascent_time/self.dt)*self.dt
        t = np.linspace(0, final_time, num=int(self.ascent_time/self.dt) + 1)
        x = np.zeros(len(t))
        y = np.zeros(len(t))
        z = (self.max_altitude / final_time) * t
        return(np.column_stack((x, y, z)))

    def hoover_portion(self):
        # For now, simple linear ascent profile
        final_time = int(self.hover_time/self.dt)*self.dt
        t = np.linspace(0, final_time, num=int(self.hover_time/self.dt))
        x = np.zeros(len(t))#(self.target[0] / final_time) * t
        y = np.zeros(len(t))#(self.target[1] / final_time) * t
        z = np.full(len(t), self.max_altitude)
        return(np.column_stack((x, y, z)))

    def descent_portion(self):
        # For now, simple linear ascent profile
        final_time = int(self.descent_time/self.dt)*self.dt
        t = np.linspace(0, final_time, num=int(self.descent_time/self.dt))
        x = np.zeros(len(t))#np.full(len(t), self.target[0])
        y = np.zeros(len(t))#np.full(len(t), self.target[1])
        z = (-1 * self.max_altitude / final_time) * t + self.max_altitude
        return(np.column_stack((x, y, z)))