import numpy as np
from alive_progress import alive_bar
from Simulator.simulation import Simulation
from Simulator.simulationConstants import *
from Graphing.GroupGUI import *
from PathPlanner.plan import PlannedTrajectory

# Setup Simulation
planned_trajectory = PlannedTrajectory(TARGET_ALTITUDE, FINAL_TIME, TIMESTEP).trajectory
sims = []
trajectories = []
number = 2
num_landed = 0
with alive_bar(number) as bar:
    for i in range(number):
        sim = Simulation(FINAL_TIME, TIMESTEP, INITIAL_STATE, WIND, planned_trajectory)
        trajectory = sim.propogate()
        sims.append(sim)
        trajectories.append(trajectory)
        if sim.landed == True:
                num_landed += 1  
        else:
                print(sim.landing_violation)
        bar()
print(num_landed/number)

# Graphs
create_group_gui(sims, planned_trajectory, trajectories, TIMESTEP, FINAL_TIME)


