import numpy as np
import control
from copy import deepcopy, copy
from Simulator.dynamics import dynamics_for_state_space_control
from Simulator.simulationConstants import GRAVITY as g
from GNC.controlConstants import *

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

def compute_A(state, u, rocket, dt):
    """ Compute Jacobian for the A matrix (State dot wrt State)
    
    Inputs:
    - state (1x12)
    - Control input U (1x3)
    - rocket object
    - timestep length
    """
    h = STEP_SIZE
    jacobian = np.zeros((len(state), len(state)))
    for i in range(len(state)):
        state_plus = deepcopy(state).astype(float)
        state_minus = deepcopy(state).astype(float)
        state_plus[i] = state_plus[i] + h
        state_minus[i] = state_minus[i] - h
        statedot_plus = dynamics_for_state_space_control(state_plus, rocket, dt, u[0], u[1], u[2])
        statedot_minus = dynamics_for_state_space_control(state_minus, rocket, dt, u[0], u[1], u[2])
        jacobian[i] = (statedot_plus - statedot_minus) / (2 * h)
    return jacobian.T

def compute_B(state, linearized_u, rocket, dt):
    """ Compute Jacobian for the B matrix (State dot wrt control input)
    
    Inputs:
    - state (1x12)
    - Control input U (1x3)
    - rocket object
    - timestep length
    """
    h = STEP_SIZE
    jacobian = np.zeros((len(linearized_u), len(state)))
    for i in range(len(linearized_u)):
        u_plus = deepcopy(linearized_u).astype(float)
        u_minus = deepcopy(linearized_u).astype(float)
        u_plus[i] = linearized_u[i] + h
        u_minus[i] = linearized_u[i] - h
        statedot_plus = dynamics_for_state_space_control(state, rocket, dt, u_plus[0], u_plus[1], u_plus[2])
        statedot_minus = dynamics_for_state_space_control(state, rocket, dt, u_minus[0], u_minus[1], u_minus[2])
        jacobian[i] = (statedot_plus - statedot_minus) / (2 * h)
    return jacobian.T

