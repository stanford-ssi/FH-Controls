import numpy as np
import control
from copy import deepcopy
from Simulator.dynamics import dynamics_for_state_space_control
from Simulator.simulationConstants import GRAVITY as g
from GNC.controlConstants import *

def state_space_control(state_error, rocket, wind, ts):
    # State Space Control
    linearized_x = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
    linearized_u = np.array([0, 0, g])
    A_orig = compute_A(linearized_x, linearized_u, rocket, wind, ts)
    B_orig = compute_B(linearized_x, linearized_u, rocket, wind, ts)
            
    # Remove Roll Columns and Rows
    A = np.delete(A_orig, 11, 0)
    A = np.delete(A, 11, 1)
    A = np.delete(A, 8, 0)
    A = np.delete(A, 8, 1)
    B = np.delete(B_orig, 11, 0)
    B = np.delete(B, 8, 0)
            
    # Q and R
    Q = np.identity(len(state_error) - 2 + 6) #Minus 2 to remove roll stuff plus 6 to get integral control
    R = np.identity(len(linearized_u))
    Q[2][2] = Q_Z_POS #Penalize Z Error
    Q[5][5] = Q_Z_VEL #Penalize Z velocity Error
    R[2][2] = R_THROTTLE #Penalize Throttle movement
            
    # Control
    C = np.zeros((6, 10))
    breakpoint()

    K,S,E = control.lqr(A, B, Q, R, integral_action=C)
    K = np.insert(K, 8, 0, axis=1)
    K = np.insert(K, 11, 0, axis=1)
    U = np.dot(-K, state_error) + linearized_u # U is the desired accelerations
    
    return U, K

def compute_A(state, u, rocket, wind, dt):
    """ Compute Jacobian for State dot wrt State"""
    h = 0.001
    jacobian = np.zeros((len(state), len(state)))
    for i in range(len(state)):
        state_plus = deepcopy(state).astype(float)
        state_minus = deepcopy(state).astype(float)
        state_plus[i] = state_plus[i] + h
        state_minus[i] = state_minus[i] - h
        statedot_plus = dynamics_for_state_space_control(state_plus, rocket, wind, dt, u[0], u[1], u[2])
        statedot_minus = dynamics_for_state_space_control(state_minus, rocket, wind, dt, u[0], u[1], u[2])
        jacobian[i] = (statedot_plus - statedot_minus) / (2 * h)
    return jacobian.T

def compute_B(state, linearized_u, rocket, wind, dt):
    """ Compute Jacobian for State dot wrt State"""
    h = 0.001
    jacobian = np.zeros((len(linearized_u), len(state)))
    for i in range(len(linearized_u)):
        u_plus = deepcopy(linearized_u).astype(float)
        u_minus = deepcopy(linearized_u).astype(float)
        u_plus[i] = linearized_u[i] + h
        u_minus[i] = linearized_u[i] - h
        statedot_plus = dynamics_for_state_space_control(state, rocket, wind, dt, u_plus[0], u_plus[1], u_plus[2])
        statedot_minus = dynamics_for_state_space_control(state, rocket, wind, dt, u_minus[0], u_minus[1], u_minus[2])
        jacobian[i] = (statedot_plus - statedot_minus) / (2 * h)
    return jacobian.T

