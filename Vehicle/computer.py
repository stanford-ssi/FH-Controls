import numpy as np
from scipy.spatial.transform import Rotation
import copy
from GNC.controller import *
from GNC.kalmanFilter import *
from GNC.constraints import *
from Simulator.dynamics import *

class FlightComputer:
    """ Class Representing the FlightComputer and associated data"""
    def __init__(self, start_state, planned_trajectory, rocket, ts):
        
        # See rocket instance it is attached to
        self.rocket_knowledge = rocket
        self.ts = ts
        
        # Rotation Matrix
        self.R = None
        self.R_history = np.array([Rotation.from_euler('xyz', [start_state[7], -start_state[6], -start_state[8]]).as_matrix()])
        
        # States and Histories
        self.start_state = start_state
        self.state = start_state
        self.state_history = np.empty((0,len(start_state)))
        self.state_previous = copy.copy(self.state)
        self.statedot_previous = np.zeros(len(start_state))
        self.ideal_trajectory = planned_trajectory
        self.error_history = np.empty((0,len(start_state)))
        
        # FFC Estimates that haven't been built into truth and ffc yet
        self.mass = self.rocket_knowledge.mass
        self.com = self.rocket_knowledge.com
        self.I = self.rocket_knowledge.I
        self.I_prev = self.rocket_knowledge.I_prev
        
        # Preform initial controller calculations
        self.U = np.array([0,0,0])
        self.u_history = np.empty((0,3))
        self.linearized_x = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
        self.linearized_u = np.array([0, 0, g])
        self.A = self.compute_A()
        self.B = self.compute_B()
        
        # Sensors:
        self.sensed_state_history = np.empty((0,16))
        self.kalman_P = np.eye(12)
               
    def rocket_loop(self, t, ts, current_step):
        
        self.mass = self.rocket_knowledge.mass
        self.com = self.rocket_knowledge.com
        self.I = self.rocket_knowledge.I
        self.I_prev = self.rocket_knowledge.I_prev
        
        # Log Rocket Rotation                  
        self.R = Rotation.from_euler('xyz', [self.state[7], -self.state[6], -self.state[8]]).as_matrix()
        self.R_history = np.vstack((self.R_history, np.expand_dims(self.R.T, axis=0)))            

        # Save truth state for sensor measurements
        truth_state = self.rocket_knowledge.state
    
        # Sense the state from sensors
        Y = np.concatenate((self.rocket_knowledge.gps.reading(truth_state, t), 
                            self.rocket_knowledge.barometer.reading(truth_state, t),
                            self.rocket_knowledge.accelerometer.reading(truth_state, self.statedot_previous[3:6], t),
                            self.rocket_knowledge.magnetometer.reading(truth_state, t),
                            self.rocket_knowledge.gyroscope.reading(truth_state, t)), axis=None)
        self.sensed_state_history = np.vstack([self.sensed_state_history, Y])
        
        self.state, self.kalman_P = kalman_filter(self.state if t > 0 else self.start_state, self.U, 
                                                    Y, self.A, self.B, self.ts, P=self.kalman_P)
        self.state_history = np.vstack([self.state_history, self.state])

        # Calculate Errors
        state_error = self.state - self.ideal_trajectory[current_step]
        self.error_history = np.vstack([self.error_history, state_error])

        # Call Controller
        self.A = self.compute_A()
        self.B = self.compute_B()
        self.K = compute_K_flight(len(self.state), self.A, self.B)
        self.U = control_rocket(self.K, state_error, self.linearized_u)
        
        #self.U = clean_control_signal(t, self.U, self.u_history)
        
        self.u_history = np.vstack([self.u_history, np.dot(self.R, self.U)]) # Rotated into rocket frame
            
    def computer_knowledge_of_dyanmics(self, state, dt, acc_x, acc_y, acc_z):
        """ These are the dynamics used during the linearization for the controller. It is the same as the regular dynamics,
            except it does not include wind forces, and assumes that the accelerations are given
            
            Inputs:
            - rocket object
            - timestep length
            - Accelerations in x, y and z
            
            Outputs:
            - statedot vector (1x12)
            """
        # Pull Params
        m = self.mass
        lever_arm = self.com
        v = state[3:6]
        w = state[9:12]
        
        # Build Statedot
        statedot = np.zeros(len(state))
        statedot[0:3] = v
        
        # Rocket rotations
        pitch = state[6] # Angle from rocket from pointing up towards positive x axis
        yaw = state[7] # Angle from rocket from pointing up towards positive y axis
        roll = state[8] # Roll, ccw when looking down on rocket
        R = Rotation.from_euler('xyz', [yaw, -pitch, -roll]).as_matrix()
        R_inv = np.linalg.inv(R)
        
        # Acceleration rotation into rocket frame
        acc = np.array([acc_x, acc_y, acc_z])
        acc_rf = np.dot(R, acc)

        # Calculate Accelerations in rocket frame
        aX_rf = acc_rf[0] + (-1 * g * R[0][2])
        aY_rf = acc_rf[1] + (-1 * g * R[1][2])
        aZ_rf = acc_rf[2] + (-1 * g * R[2][2])
        a_rf = np.array([aX_rf, aY_rf, aZ_rf])

        # Convert Accelerations from rocket frame to global frame
        a_global = np.dot(R_inv, a_rf)

        # Calculate Alphas
        torque = np.array([(acc_rf[0] * m * lever_arm),
                            (acc_rf[1] * m * lever_arm),
                            0])
        I_dot = (self.I - self.I_prev) / dt
        alphas = np.dot(np.linalg.inv(self.I), torque - np.cross(w, np.dot(self.I, w)) - np.dot(I_dot, w))

        statedot[3:6] = a_global.tolist()
        statedot[9:12] = alphas.tolist()
        
        # Rotational Kinematics
        statedot[6:9] = get_EA_dot(state)
        return statedot
    
    def compute_A(self):
        """ Compute Jacobian for the A matrix (State dot wrt State)
        
        Inputs:
        - state (1x12)
        - Control input U (1x3)
        - rocket object
        - timestep length
        """
        h = STEP_SIZE
        jacobian = np.zeros((len(self.state), len(self.state)))
        for i in range(len(self.state)):
            state_plus = deepcopy(self.state).astype(float)
            state_minus = deepcopy(self.state).astype(float)
            state_plus[i] = state_plus[i] + h
            state_minus[i] = state_minus[i] - h
            statedot_plus = self.computer_knowledge_of_dyanmics(state_plus, self.ts, self.linearized_u[0], self.linearized_u[1], self.linearized_u[2])
            statedot_minus = self.computer_knowledge_of_dyanmics(state_minus, self.ts, self.linearized_u[0], self.linearized_u[1], self.linearized_u[2])
            jacobian[i] = (statedot_plus - statedot_minus) / (2 * h)
        return jacobian.T

    def compute_B(self):
        """ Compute Jacobian for the B matrix (State dot wrt control input)
        
        Inputs:
        - state (1x12)
        - Control input U (1x3)
        - rocket object
        - timestep length
        """
        h = STEP_SIZE
        jacobian = np.zeros((len(self.linearized_u), len(self.state)))
        for i in range(len(self.linearized_u)):
            u_plus = deepcopy(self.linearized_u).astype(float)
            u_minus = deepcopy(self.linearized_u).astype(float)
            u_plus[i] = self.linearized_u[i] + h
            u_minus[i] = self.linearized_u[i] - h
            statedot_plus = self.computer_knowledge_of_dyanmics(self.state, self.ts, u_plus[0], u_plus[1], u_plus[2])
            statedot_minus = self.computer_knowledge_of_dyanmics(self.state, self.ts, u_minus[0], u_minus[1], u_minus[2])
            jacobian[i] = (statedot_plus - statedot_minus) / (2 * h)
        return jacobian.T
