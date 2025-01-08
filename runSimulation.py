import numpy as np
from Simulator.simulation import Simulation
from Simulator.simulationConstants import *
from Graphing.GUI import *
from PathPlanner.plan import PlannedTrajectory
from Graphing.monocole import *

import csv
import shutil
from tkinter import Tk
from tkinter.filedialog import asksaveasfilename

# Setup Simulation
planned_trajectory = PlannedTrajectory(TARGET_ALTITUDE, FINAL_TIME, TIMESTEP).trajectory
sim = Simulation(FINAL_TIME, TIMESTEP, INITIAL_STATE, WIND, planned_trajectory)

# Run Simulation
trajectory = sim.propogate()
sim.display_end_info()

attributes = {
    "Altitude History": sim.rocket.state_history[:, 2],
    "Altitude FFC History": sim.rocket.ffc.state_history[:, 2],
    "X Truth position_error": sim.rocket.error_history[:, 0],
    "Y Truth position error": sim.rocket.error_history[:, 1],
    "Z Truth position error": sim.rocket.error_history[:, 2],
    "X FFC position error": sim.rocket.ffc.error_history[:, 0],
    "Y FFC position_error": sim.rocket.ffc.error_history[:, 1],
    "Z FFC position_error": sim.rocket.ffc.error_history[:, 2],
    "Pitch Truth Error": sim.rocket.error_history[:,6] * RAD2DEG,
    "Yaw Truth Error": sim.rocket.error_history[:,7] * RAD2DEG,
    "Roll Truth Error": sim.rocket.error_history[:,8] * RAD2DEG,
    "Pitch FFC Error": sim.rocket.ffc.error_history[:,6] * RAD2DEG,
    "Yaw FFC Error": sim.rocket.ffc.error_history[:,7] * RAD2DEG,
    "Roll FFC Error": sim.rocket.ffc.error_history[:,8] * RAD2DEG, 
    "Gimbal Theta": np.arctan2(sim.rocket.engine.posy_history,sim.rocket.engine.posx_history) * RAD2DEG,
    "Gimbal Psi": np.arctan2(np.sqrt((sim.rocket.engine.posx_history ** 2) + (sim.rocket.engine.posy_history ** 2)), sim.rocket.engine.length) * RAD2DEG,
    "Throttle": sim.rocket.engine.throttle_history,
    "Gimbal X Position": sim.rocket.engine.posx_history,
    "Gimbal Y Position":sim.rocket.engine.posy_history,
    "Control Input X": sim.rocket.ffc.u_history[:,0],
    "Control Input y": sim.rocket.ffc.u_history[:,1],
    "Control Input Z": sim.rocket.ffc.u_history[:,2],
    "Moment of Inertia XX": [arr[0,0] for arr in sim.rocket.I_history],
    "Moment of Inertia YY": [arr[1,1] for arr in sim.rocket.I_history],
    "Moment of Inertia ZZ": [arr[2,2] for arr in sim.rocket.I_history],
    "State History": sim.rocket.state_history
}

with open("attributes.csv", "w", newline="") as file:
    writer = csv.writer(file)
    # Write key-value pairs
    for key, value in attributes.items():
        writer.writerow([key, list(value)])  # Convert ndarray to list

Tk().withdraw()
file_path = asksaveasfilename(
    defaultextension=".csv",
    filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
    title= "Choose location to save your file"
)
if file_path:
    shutil.move("attributes.csv", file_path)
    print(f"File moved to {file_path}.")
else:
    print("Save operation canceled.")

# Graphs
create_gui(sim, planned_trajectory, trajectory, TIMESTEP, sim.previous_time)
#plot_frames_over_time(sim)



