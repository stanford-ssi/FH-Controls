import numpy as np
from Simulator.simulation import Simulation
import Graphing.plotter
from PathPlanner.plan import PlannedTrajectory

# Simulation Variables
# State is all relative to global frame except Z rotation which is rocket frame
state_0 = np.array([0.1, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0]) # Start State [X, Y, Z, VX, VY, VZ, THX, THY, THZ, OMX, OMY, OMZ]
tf = 10
ts = 0.1

# Setup Simulation
planned_trajectory = PlannedTrajectory(50, tf, [0, 0, 0], ts).trajectory
sim = Simulation(tf, ts, state_0, planned_trajectory)
# Run Simulation
trajectory = sim.propogate()
sim.display_end_info()

# Pull Info for Graphs
position_error = [sim.position_error_history[:,0], sim.position_error_history[:,1], sim.position_error_history[:,2]]
error_names = ["X Error", "Y Error", "Z Error"]
rotation_error = [sim.rotation_error_history[:,0], sim.rotation_error_history[:,1], sim.rotation_error_history[:,2]]
rot_error_names = ["Pitch Error", "Yaw Error", "Roll Error"]
controls = [sim.rocket.engine.posx_history, sim.rocket.engine.posy_history, sim.rocket.engine.throttle_history]
control_names = ["PosX", "PosY", "Throttle"]
dynamics = Graphing.plotter.dynamics(trajectory, ts, tf)

dynamics_plot_names = ["X Position", "Y Position", "Z Position", "X Velocity", "Y Velocity", "Z Velocity", "X Acceleration", "Y Acceleration", "Z Acceleration"]
rotational_dynamics_plot_names = ["THX", "THY", "THZ", "OMX", "OMY", "OMZ", "ALPHAX", "ALPHAY", "ALPHAZ"]

# Graphs
Graphing.plotter.animate_3DOF_trajectory(trajectory, planned_trajectory)
Graphing.plotter.plot_3(position_error, ts, tf, error_names)
Graphing.plotter.plot_3(rotation_error, ts, tf, rot_error_names)
Graphing.plotter.plot_3(controls, ts, tf, control_names)
Graphing.plotter.plot_dynamics(dynamics[0:9], ts, tf, names=dynamics_plot_names)
Graphing.plotter.plot_dynamics(dynamics[9:18], ts, tf, names=rotational_dynamics_plot_names)






