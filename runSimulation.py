import numpy as np
from Simulator.simulation import Simulation
import Graphing.plotter
from PathPlanner.plan import PlannedTrajectory 
import PathPlanner.pathPlannerConstants 

# Simulation Variables
state_0 = np.array([0, 0, 0, 0, 0, 0]) # Start State
tf = 10
ts = 0.1

#forgive me for I have sinned 
g = 9.81  # m/s^2, acceleration due to gravity
dt = 0.2  # seconds, time step
N = 100  # number of time steps

# Derived constants
max_a_rate_up = 0.1 * g * dt # m/s^2, maximum upward acceleration
max_a_rate_down = -0.1 * g * dt # m/s^2, maximum downward acceleration 
h_max = 55  # m, maximum height
h_min = 0  # m, minimum height
v_max = 10  # m/s, maximum velocity
v_min = -10  # m/s, minimum velocity
a_max = g  # m/s^2, maximum acceleration
a_min = -g  # m/s^2 minimum acceleration
h_target = 50  # m, target height
v_target = 0  # m/s, target velocity
x0 = np.array([0, 0, 0])  # [m, m, m], initial state vector

# Model matrices
Phi = np.array([[1, dt], [0, 1]])
Gamma = np.array([0, dt])







# Setup Simulation

#variables used for planned trajectory come from pathPlannerConstants (this could cause issues, and is something I want to rework)
planned_trajectory = PlannedTrajectory(N, dt, x0, h_target, v_target, h_max, h_min, v_max, v_min, a_max, a_min, max_a_rate_up, max_a_rate_down).solve_optimization_problem()
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






