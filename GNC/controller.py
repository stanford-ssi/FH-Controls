import numpy as np
import control
from copy import deepcopy, copy
from Simulator.dynamics import *
from Simulator.simulationConstants import GRAVITY as g
from GNC.controlConstants import *
import cvxpy as cvx
from Vehicle.computer import *

def control_rocket(K, state_error, linearized_u):
    """ Function that is called to get control inputs at each time step
    
    Inputs:
    - Controller K matrix
    - State error in global frame
    - linearized control input 
    
    Output:
    - Control Input Vector U
    """
    U = np.dot(-K, np.append(state_error, state_error[0:3])) + linearized_u # U is the desired accelerations
    return U

def update_linearization(A_old, B_old, R):
    """ Update linearization to account for rocket rotation
    
    Inputs:
    - Old A matrix
    - Old B matrix
    - Rocket rotation matrix R
    
    Output
    - rotated A 
    - rotated B
    
    """
    
    A = copy(A_old)
    B = copy(B_old)
    A[9:12,6:9] = np.dot(R, A[9:12,6:9])
    A[9:12,9:12] = np.dot(R, A[9:12,9:12])
    B[9:12,0:3] = np.dot(R, B[9:12,0:3])
    return A, B

def get_QR_LQR(len_state, len_u):
    # Q and R
    Q = np.identity(len_state - 2 + 3) #Minus 2 to remove roll stuff plus 3 to get integral control
    R = np.identity(len_u)

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
    return Q, R

def compute_K_flight(len_state, A, B):
    """Compute the K matrix for the flight phase of the mission using lqr
    
    Inputs:
    - length of state
    - A matrix
    - B matrix 
    """
    
    linearized_u = np.array([0, 0, g])
            
    # Remove Roll Columns and Rows
    A = np.delete(A, 11, 0)
    A = np.delete(A, 11, 1)
    A = np.delete(A, 8, 0)
    A = np.delete(A, 8, 1)
    B = np.delete(B, 11, 0)
    B = np.delete(B, 8, 0)
            
    Q, R = get_QR_LQR(len_state, len(linearized_u))
            
    # Control
    C = np.append(np.identity(3), np.zeros((3, 7)), axis=1) # C is of the form y = Cx, where x is the state and y is the steady state error we care about - in this case just [x y z]
   
    K,S,E = control.lqr(A, B, Q, R, integral_action=C) # The K that this spits out has the additional 3 integral terms tacked onto the end, so must add errorx, y, z onto end of state error when solving for U
    K = np.insert(K, 8, 0, axis=1)
    K = np.insert(K, 11, 0, axis=1)
    
    return K

def low_pass_filter(signal, previous_signal_filtered, alpha):
    filtered_value = alpha * signal + (1 - alpha) * previous_signal_filtered
    return filtered_value

def clean_control_signal(t, U, u_history):
    if not t == 0:
        U[0] = low_pass_filter(U[0], u_history[-1][0], 1)
        U[1] = low_pass_filter(U[1], u_history[-1][1], 1)
        U[2] = low_pass_filter(U[2], u_history[-1][2], 0.75)
        
    return U