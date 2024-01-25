import numpy as np
import control
from copy import deepcopy
from Simulator.dynamics import dynamics_for_state_space_control
from Simulator.simulationConstants import GRAVITY as g
from GNC.controlConstants import *

def control_rocket(K, state_error, linearized_u):
    U = np.dot(-K, np.append(state_error, state_error[0:3])) + linearized_u # U is the desired accelerations
    return U

def compute_K(len_state, A_orig, B_orig):
    """State Space Control"""
    
    linearized_u = np.array([0, 0, g])
            
    # Remove Roll Columns and Rows
    A = np.delete(A_orig, 11, 0)
    A = np.delete(A, 11, 1)
    A = np.delete(A, 8, 0)
    A = np.delete(A, 8, 1)
    B = np.delete(B_orig, 11, 0)
    B = np.delete(B, 8, 0)
            
    # Q and R
    Q = np.identity(len_state - 2 + 3) #Minus 2 to remove roll stuff plus 3 to get integral control
    R = np.identity(len(linearized_u))

    Q[0][0] = 1 / (Q_X ** 2)
    Q[1][1] = 1 / (Q_Y ** 2)
    Q[2][2] = 1 / (Q_Z ** 2)
    Q[3][3] = 1 / (Q_VX ** 2)
    Q[4][4] = 1 / (Q_VY ** 2)
    Q[5][5] = 1 / (Q_VZ ** 2)
    Q[6][6] = 1 / (Q_PIT ** 2)
    Q[7][7] = 1 / (Q_YAW ** 2)
    Q[8][8] = 1 / (Q_VPIT ** 2)
    Q[9][9] = 1 / (Q_VYAW ** 2)
    Q[10][10] = 1 / (Q_X ** 2)
    Q[11][11] = 1 / (Q_Y ** 2)
    Q[12][12] = 1 / (Q_Z ** 2)
    
    R[0][0] = 1 / (R_X ** 2)
    R[1][1] = 1 / (R_Y ** 2)
    R[2][2] = 1 / (R_T ** 2)
            
    # Control
    C = np.append(np.identity(3), np.zeros((3, 7)), axis=1) # C is of the form y = Cx, where x is the state and y is the steady state error we care about - in this case just [x y z]
    K,S,E = control.lqr(A, B, Q, R, integral_action=C) # The K that this spits out has the additional 3 integral terms tacked onto the end, so must add errorx, y, z onto end of state error when solving for U
    K = np.insert(K, 8, 0, axis=1)
    K = np.insert(K, 11, 0, axis=1)
    
    return K

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

