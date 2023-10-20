"""Account for all forces acting on Object. 3DOF for now, will upgrade to 6DOF later"""

import numpy as np
import Rocket.thrust

# Constants
g = 9.81

def state_to_stateDot(state, t):

    # Initialize
    statedot = np.zeros(6)

    # Move Velocity
    statedot[0:3] = state[3:6]

    # Get Timebased Parameters
    T = Rocket.thrust.getThrust(t)
    
    # Calculate Acceleration
    r = state[0:2]
    statedot[3:6] = [0,0,T-g]
    return statedot

