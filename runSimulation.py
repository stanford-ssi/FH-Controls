import numpy as np
from Simulator.simulation import Simulation
from Simulator.simulationConstants import *
from Graphing.GUI import *
from PathPlanner.plan import PlannedTrajectory
from Graphing.monocole import *
from app import *


# Setup Simulation
planned_trajectory = PlannedTrajectory(TARGET_ALTITUDE, FINAL_TIME, TIMESTEP).trajectory
sim = Simulation(FINAL_TIME, TIMESTEP, INITIAL_STATE, WIND, planned_trajectory)

# Run Simulation
trajectory = sim.propogate()
sim.display_end_info()

# Graphs
run_dash(sim, planned_trajectory, trajectory, TIMESTEP, sim.previous_time)
plot_frames_over_time(sim)
