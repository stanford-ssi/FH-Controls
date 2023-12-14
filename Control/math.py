import numpy as np
from copy import deepcopy
from Simulator.dynamics import natural_dyanamics

def compute_Jacobian(state, rocket, wind, dt):
    """ Compute Jacobian for State dot wrt State"""
    h = 0.001
    jacobian = np.zeros((len(state), len(state)))
    for i in range(len(state)):
        state_plus = deepcopy(state)
        state_minus = deepcopy(state)
        state_plus[i] = state_plus[i] + h
        state_minus[i] = state_minus[i] - h
        statedot_plus = natural_dyanamics(state_plus, rocket, wind, dt)
        statedot_minus = natural_dyanamics(state_minus, rocket, wind, dt)
        jacobian[i] = (statedot_plus - statedot_minus) / (2 * h)
    return jacobian



