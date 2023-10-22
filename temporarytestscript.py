import numpy as np
import Simulator.ode
from Simulator.simulation import Simulation
import Graphing.plotter
from PathPlanner.plan import PlannedTrajectory


# planned_trajectory = PlannedTrajectory(5, 50, 5, 5, [25, 25, 0], 0.1)

state_0 = np.array([0, 0, 0, 5, 0, 20])
tf = 5
ts = 0.1


trajectory = Simulation(tf, ts, state_0).propogate()
Graphing.plotter.plot_3DOF_trajectory(trajectory)

# CURRENT WORK
# 1. Create simulation object, which then creates a rocket object and propogates



