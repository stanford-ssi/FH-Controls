import numpy as np
from Simulator.simulation import Simulation
from Graphing.GUI import *
from PathPlanner.plan import PlannedTrajectory

# Simulation Variables
# State is all relative to global frame except Z rotation which is rocket frame
# Pitch is defined as angle from upward z axis towards pos x axis, yaw is angle from upward z towards pos y, and roll is ccw looking down on rocket
# Rotation order yaw, pitch, roll
state_0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]) # Start State [X, Y, Z, VX, VY, VZ, THX, THY, THZ, OMX, OMY, OMZ]
wind = np.array([5, 5, 0])
tf = 20
ts = 0.1
max_altitude = 50

# Setup Simulation
planned_trajectory = PlannedTrajectory(max_altitude, tf, ts).trajectory
sim = Simulation(tf, ts, state_0, wind, planned_trajectory)

# Run Simulation
trajectory = sim.propogate()
sim.display_end_info()

# Graphs
create_gui(sim, planned_trajectory, trajectory, ts, tf)
