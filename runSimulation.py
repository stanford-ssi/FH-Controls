import numpy as np
from Simulator.simulation import Simulation
import Graphing.plotter
from Graphing.GUI import *
from PathPlanner.plan import PlannedTrajectory

# Simulation Variables
# State is all relative to global frame except Z rotation which is rocket frame
# Pitch is defined as angle from upward z axis towards pos x axis, yaw is angle from upward z towards pos y, and roll is ccw looking down on rocket
# Rotation order yaw, pitch, roll
state_0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]) # Start State [X, Y, Z, VX, VY, VZ, THX, THY, THZ, OMX, OMY, OMZ]
wind = np.array([1, 5, 0.001])
tf = 20
ts = 0.1
max_altitude = 50

# Setup Simulation
planned_trajectory = PlannedTrajectory(max_altitude, tf, ts).trajectory

sim = Simulation(tf, ts, state_0, wind, planned_trajectory)

# Run Simulation
trajectory = sim.propogate()
sim.display_end_info()

# Pull Info for Graphs







# Graphs
create_gui(sim, trajectory, ts, tf)
#PlannedTrajectory(max_altitude, tf, ts).plot_trajectory()
#Graphing.plotter.animate_3DOF_trajectory(trajectory, planned_trajectory)
#Graphing.plotter.plot_3(position_error, ts, tf, error_names)
#Graphing.plotter.plot_3(rotation_error, ts, tf, rot_error_names)
#Graphing.plotter.plot_3(controls, ts, tf, control_names)
#Graphing.plotter.plot_variables_vs_time([planned_trajectory[:,2], trajectory[:,2], controls[2] * 10], ts, tf, name="Altitude")
#Graphing.plotter.plot_3(moi, ts, tf, moi_names)
#Graphing.plotter.plot_dynamics(dynamics[0:9], ts, tf, names=dynamics_plot_names)
#Graphing.plotter.plot_dynamics(dynamics[9:18], ts, tf, names=rotational_dynamics_plot_names)
