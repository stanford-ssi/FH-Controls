import numpy as np
import random
import copy
from Simulator.simulationConstants import *

def actuator_error_injection(pos_x, pos_y, throttle):
    """ Inject error into the acutator positions. The parameters governing the gaussian are from the simulation constants file
    
    Inputs:
    - pos_x
    - pos_y
    - throttle
    
    Outputs:
    - modified pos_x
    - modified pos_y
    - modified throttle
    
    """
    pos_x = np.random.normal(pos_x + np.random.normal(0, RANDOMIZED_ERROR_POS_SIGMA * 10), RANDOMIZED_ERROR_POS_SIGMA)
    pos_y = np.random.normal(pos_y + np.random.normal(0, RANDOMIZED_ERROR_POS_SIGMA * 10), RANDOMIZED_ERROR_POS_SIGMA)
    throttle = np.random.normal(throttle + CONSTANT_ERROR_THROTTLE, RANDOMIZED_ERROR_THROTTLE_SIGMA)
    return pos_x, pos_y, throttle

def roll_injection(state):
    """ Inject roll into the rocket. The parameters governing the gaussian are from the simulation constants file"""
    new_state = copy.copy(state).astype(float)
    random = np.random.normal(ROLL_MU, ROLL_SIGMA)
    new_state[11] = random
    return new_state

def wind_randomness(base_wind, current_wind):
    """ Function that gets the wind at the current timestep. It randomizes the gusts up to 1.5 times the base wind speed.
        It's randomization is based on two variables defined in the simulation constants wind section
        
        Inputs:
        - base_wind 1x3 array
        - current wind 1x3 array
        
        Outputs:
        - new_wind 1x3 array
        """
        
    if (base_wind==current_wind).all():
        if random.random() > WIND_CHANCE_OF_CHANGE:
            multiplier = WIND_MULTIPLIER * random.random()
            return base_wind * multiplier
        else:
            return base_wind
    else:
        if random.random() > WIND_CHANGE_LENGTH:
            return base_wind
        else:
            return current_wind