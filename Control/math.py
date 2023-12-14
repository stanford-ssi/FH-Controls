import numpy as np

def compute_Jacobian(state, stateDot):
    """ Compute Jacobian for State dot wrt State"""
    jacobian = np.zeros((len(state), len(state)))
    for i in range(len(state)):
        for j in range(len(state)):
            
