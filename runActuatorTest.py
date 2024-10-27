import numpy as np
from Simulator.simulation import Simulation
from Simulator.simulationConstants import *
from Graphing.GroupGUI import *
from PathPlanner.plan import PlannedTrajectory
import matplotlib.pyplot as plt


# Setup Simulation
planned_trajectory = PlannedTrajectory(TARGET_ALTITUDE, FINAL_TIME, TIMESTEP).trajectory
T = np.linspace(0, 0.25, 10)
K = np.linspace(0, 5, 10)
result = []
for value in K:
    small_result = []
    for value in T:
        sims = []
        trajectories = []
        number = 25
        num_landed = 0
        for i in range(number):
            sim = Simulation(FINAL_TIME, TIMESTEP, INITIAL_STATE, WIND, planned_trajectory)
            sim.rocket.actuator_X.refresh(1, value)     
            sim.rocket.actuator_Y.refresh(1, value) 
            trajectory = sim.propogate()
            sims.append(sim)
            trajectories.append(trajectory)
            if sim.landed == True:
                    num_landed += 1  
            else:
                    print(sim.landing_violation)
        landing_rate = num_landed/number
        small_result.append(landing_rate)
    result.append(small_result)
    
# Convert Z to a numpy array
Z = np.array(result)
X, Y = np.meshgrid(K, T)

# Create the contour plot
plt.contour(X, Y, Z, levels=10, cmap='viridis')

# Add labels and title
plt.title('Contour Plot')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')

# Add a colorbar
plt.colorbar()

# Display the plot
plt.show()
    