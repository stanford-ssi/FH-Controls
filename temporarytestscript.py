import numpy as np
import Simulator.ode
import Simulator.propogator
import Graphing.plotter
from PathPlanner.plan import PlannedTrajectory


planned_trajectory = PlannedTrajectory(5, 50, 5, 5, [25, 25, 0], 0.1)

state_0 = np.array([0, 0, 0, 5, 0, 20])
tf = 5
ts = 0.1


trajectory = Simulator.propogator.propogate(state_0, tf, ts)
Graphing.plotter.plot_3DOF_trajectory(trajectory)


