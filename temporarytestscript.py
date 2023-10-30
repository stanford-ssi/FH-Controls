import numpy as np
from Simulator.simulation import Simulation
import Graphing.plotter
from PathPlanner.plan import PlannedTrajectory



# State is of form x y z xdot ydot zdot totalmass throttle
state_0 = np.array([0, 0, 0, 0, 0, 0])
tf = 15
ts = 0.1

planned_trajectory = PlannedTrajectory(tf/3, 50, tf/3, tf/3, [25, 25, 0], ts).trajectory
sim = Simulation(tf, ts, state_0, planned_trajectory)

trajectory = sim.propogate()
mass = sim.rocket.massHistory
throttle = sim.rocket.engine.throttle_history
Graphing.plotter.animate_3DOF_trajectory(trajectory, planned_trajectory)
Graphing.plotter.plot_error(errorHistory=sim.errorHistory)
Graphing.plotter.plot_mass(mass, ts, tf)


