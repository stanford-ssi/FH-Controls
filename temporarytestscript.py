import numpy as np
from Simulator.simulation import Simulation
import Graphing.plotter
from PathPlanner.plan import PlannedTrajectory

# Simulation Variables
state_0 = np.array([0, 0, 0, 0, 0, 0]) # Start State
tf = 10
ts = 0.1

# Setup Simulation
planned_trajectory = PlannedTrajectory(2*tf/5, 50, 3*tf/5, tf/5, [25, 25, 0], ts).trajectory
sim = Simulation(tf, ts, state_0, planned_trajectory)

# Run Simulation
trajectory = sim.propogate()
sim.display_end_info()

# Pull Info for Graphs
mass = sim.rocket.massHistory
throttle = sim.rocket.engine.throttle_history
dynamics = Graphing.plotter.dynamics(trajectory, ts, tf)

# Graphs
"""
Graphing.plotter.animate_3DOF_trajectory(trajectory, planned_trajectory)
Graphing.plotter.plot_variable_vs_time(sim.errorHistory[:,2], ts, tf, name="Z Error (m)")
Graphing.plotter.plot_variable_vs_time(mass, ts, tf, name="Mass (kg)")
Graphing.plotter.plot_variable_vs_time(throttle, ts, tf, name="Throttle")
Graphing.plotter.plot_variable_vs_time(dynamics[0], ts, tf, name="X Position")
Graphing.plotter.plot_variable_vs_time(dynamics[1], ts, tf, name="Y Position")
Graphing.plotter.plot_variable_vs_time(dynamics[2], ts, tf, name="Z Position")
Graphing.plotter.plot_variable_vs_time(dynamics[3], ts, tf, name="X Velocity")
Graphing.plotter.plot_variable_vs_time(dynamics[4], ts, tf, name="Y Velocity")
Graphing.plotter.plot_variable_vs_time(dynamics[5], ts, tf, name="Z Velocity")
Graphing.plotter.plot_variable_vs_time(dynamics[6], ts, tf, name="X Acceleration")
Graphing.plotter.plot_variable_vs_time(dynamics[7], ts, tf, name="Y Acceleration")
Graphing.plotter.plot_variable_vs_time(dynamics[8], ts, tf, name="Z Acceleration")

"""






