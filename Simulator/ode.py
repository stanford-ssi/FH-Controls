"""Account for all forces acting on Object. 3DOF for now, will upgrade to 6DOF later"""

import numpy as np
import Vehicle.engine
import Vehicle.rocket

# Constants
g = 9.81

def wrapper_state_to_stateDot(t, state, rocket, ideal_trajectory, t_vec):
    global previous_time
    global currStep
    if t == 0:
        currStep = 0

    # Check if we are on an actual simulation timestep or if this is ode solving shenanigans
    if (t == 0) or (t >= t_vec[currStep] and previous_time < t_vec[currStep]):
            
        # FIND ACTUATOR ADJUSTMENTS
        #ideal_state = ideal_trajectory[idx]
        throttle = 0.6
        # theta_x = blah
        # theta_y = bleh

        # Log Current States
        rocket.engine.save_throttle(throttle)
        rocket.engine.save_thrust(rocket.engine.get_thrust(t, throttle))
        rocket.update_mass(t)

        if not t == t_vec[-1]:
            currStep += 1

    previous_time = t
    return state_to_stateDot(t, state, rocket)


def state_to_stateDot(t, state, rocket):

    # Pull Params
    throttle = rocket.engine.throttle
    T = rocket.engine.get_thrust(t, throttle)
    m = rocket.mass
    
    # Build Statedot
    statedot = np.zeros(6)
    statedot[0:3] = state[3:6]

    # Calculate Acceleration
    statedot[3:6] = [0,0, (T / m) - g]
    return statedot

