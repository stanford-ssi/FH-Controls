import numpy as np
from Simulator.simulation import Simulation
import Graphing.plotter
from PathPlanner.plan import PlannedTrajectory



# State is of form x y z xdot ydot zdot
state_0 = np.array([0, 0, 0, 0, 0, 0])
tf = 10
ts = 0.1

planned_trajectory = PlannedTrajectory(2*tf/5, 50, 3*tf/5, tf/5, [25, 25, 0], ts).trajectory
sim = Simulation(tf, ts, state_0, planned_trajectory)

trajectory = sim.propogate()

mass = sim.rocket.massHistory
throttle = sim.rocket.engine.throttle_history
print(mass)
print(throttle)
#Graphing.plotter.animate_3DOF_trajectory(trajectory, planned_trajectory)
Graphing.plotter.plot_Z_error(errorHistory=sim.errorHistory)
Graphing.plotter.plot_variable_vs_time(mass, ts, tf, name="Mass (kg)")
Graphing.plotter.plot_variable_vs_time(throttle, ts, tf, name="Throttle")


