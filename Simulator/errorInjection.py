import numpy as np
import random
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