import numpy as np
import random
import copy
from Simulator.simulationConstants import *

def actuator_error_injection(pos_x, pos_y, throttle):
    # Add some randomized error
    pos_x = pos_x + (random.randint(-10, 10) * 0.1 * RANDOMIZED_ERROR_POS) + CONSTANT_ERROR_POS
    pos_y = pos_y + (random.randint(-10, 10) * 0.1 * RANDOMIZED_ERROR_POS) + CONSTANT_ERROR_POS
    throttle = throttle + (random.randint(-10, 10) * 0.1 * RANDOMIZED_ERROR_THROTTLE) + CONSTANT_ERROR_THROTTLE
    return pos_x, pos_y, throttle

def state_error_injection(state):
    noise = np.random.normal(STATE_MU, STATE_SIGMA, len(state))
    return state + noise

def roll_injection(state):
    new_state = copy.copy(state).astype(float)
    random = np.random.normal(ROLL_MU, ROLL_SIGMA)
    new_state[11] = random
    return new_state

def wind_randomness(base_wind, current_wind):
    """ Function that gets the wind at the current timestep. It randomizes the gusts up to 5 times the base wind speed.
        It's randomization is based on two variables defined in the simulation constants wind section
        
        Inputs:
        - base_wind 1x3 array
        - current wind 1x3 array
        
        Outputs:
        - new_wind 1x3 array
        """
        
    if (base_wind==current_wind).all():
        if random.random() > WIND_CHANCE_OF_CHANGE:
            multiplier = 2 * random.random()
            return base_wind * multiplier
        else:
            return base_wind
    else:
        if random.random() > WIND_CHANGE_LENGTH:
            return base_wind
        else:
            return current_wind