import numpy as np
from Simulator.simulation import Simulation
import Graphing.plotter
from PathPlanner.plan import PlannedTrajectory

# Simulation Variables
state_0 = np.array([0, 0, 0, 0, 0, 0]) # Start State
tf = 10
ts = 0.1


planned_trajectory = PathPlanner(N, dt, x0, h_target, v_target, h_max, h_min, v_max, v_min, a_max, a_min, max_a_rate_up, max_a_rate_down)
planned_trajectory.solve_optimization_problem()
planner.plot_results()


# Setup Simulation
planned_trajectory = PlannedTrajectory(50, tf, [0, 0, 0], ts).trajectory
sim = Simulation(tf, ts, state_0, planned_trajectory)
# Run Simulation
trajectory = sim.propogate()
sim.display_end_info()
# Pull Info for Graphs
# JERRY: we'll plot error, mass, and throttle on one window
# Then, the dynamics information on a second window
error_mass_throttle = [sim.errorHistory[:,2], sim.rocket.massHistory, sim.rocket.engine.throttle_history]
dynamics = Graphing.plotter.dynamics(trajectory, ts, tf)
dynamics_plot_names = ["X Position", "Y Position", "Z Position", "X Velocity", "Y Velocity", "Z Velocity", "X Acceleration", "Y Acceleration", "Z Acceleration"]

# Graphs
Graphing.plotter.animate_3DOF_trajectory(trajectory, planned_trajectory)
Graphing.plotter.plot_error_mass_throttle(error_mass_throttle, ts, tf)
Graphing.plotter.plot_dynamics(dynamics, ts, tf, names=dynamics_plot_names)






