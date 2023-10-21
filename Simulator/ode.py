"""Account for all forces acting on Object. 3DOF for now, will upgrade to 6DOF later"""

import numpy as np
import Vehicle.engine
import Vehicle.mass
import Vehicle.rocket

# Constants
g = 9.81

def state_to_stateDot(state, t, rocket: Vehicle.rocket.Rocket):

    # Initialize
    statedot = np.zeros(6)

    # Update Parameters:
    throttle = 0.75
    # theta_x = blah
    # theta_y = bleh

    # Thrust
    rocket.engine.save_throttle()
    T = rocket.engine.get_thrust(t, throttle)

    # Mass
    m = rocket.get_mass()
    
    # Move Velocity
    statedot[0:3] = state[3:6]

    # Calculate Acceleration
    r = state[0:2]
    statedot[3:6] = [0,0, (T / m) - g]
    return statedot

