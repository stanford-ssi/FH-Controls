import numpy as np
from scipy.spatial.transform import Rotation
import copy
from Vehicle.controlConstants import *
from Simulator.constraints import *
from Simulator.dynamics import *
import Vehicle.engineConstants
from copy import deepcopy
import control
import math

class FlightComputer:
    """ Class Representing the FlightComputer and associated data"""
    def __init__(self, start_state, planned_trajectory, rocket, ts):
        
        # See rocket instance it is attached to
        self.rocket_knowledge = rocket
        self.ts = ts
        self.t = 0
        
        # States and Histories
        self.start_state = start_state
        self.state = start_state
        self.state_history = np.empty((0,len(start_state)))
        self.state_previous = copy.copy(self.state)
        self.statedot_previous = np.zeros(len(start_state))
        self.ideal_trajectory = planned_trajectory
        self.error_history = np.empty((0,len(start_state)))
        
        # Rotation Matrix
        self.R = Rotation.from_euler('xyz', [self.state[7], -self.state[6], -self.state[8]]).as_matrix()
        self.R_history = np.array([Rotation.from_euler('xyz', [start_state[7], -start_state[6], -start_state[8]]).as_matrix()])
        
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
        
        # Engine States:
        self.throttle_history = np.empty(shape=(0))
        self.throttle = 1
        self.posx_history = np.empty(shape=(0))
        self.posx = 0
        self.posy_history = np.empty(shape=(0))
        self.posy = 0
        
        # Known Rocket Info
        self.engine_length = Vehicle.engineConstants.LENGTH
        self.burn_duration =  Vehicle.engineConstants.ENGINE_BURN_DURATION
        self.thrust_curve = Vehicle.engineConstants.THRUST_CURVE
        self.dt_thrust_curve = Vehicle.engineConstants.DT_THRUST_CURVE
               
    def rocket_loop(self, t, current_step):
        
        self.t = t
        
        self.mass = self.rocket_knowledge.mass
        self.com = self.rocket_knowledge.com
        self.I = self.rocket_knowledge.I
        self.I_prev = self.rocket_knowledge.I_prev
        
        # Log Rocket Rotation                  
        self.R = Rotation.from_euler('xyz', [self.state[7], -self.state[6], -self.state[8]]).as_matrix()
        self.R_history = np.vstack((self.R_history, np.expand_dims(self.R.T, axis=0)))            

        # Save truth state for sensor measurements
        truth_state = self.rocket_knowledge.state
        self.A = self.compute_A()
        self.B = self.compute_B()
    
        # Sense the state from sensors
        self.measurement = np.concatenate((self.rocket_knowledge.gps.reading(truth_state, t), 
                            self.rocket_knowledge.barometer.reading(truth_state, t),
                            self.rocket_knowledge.accelerometer.reading(truth_state, self.statedot_previous[3:6], t),
                            self.rocket_knowledge.magnetometer.reading(truth_state, t),
                            self.rocket_knowledge.gyroscope.reading(truth_state, t)), axis=None)
        self.sensed_state_history = np.vstack([self.sensed_state_history, self.measurement])
        self.kalman_filter()
        self.state_history = np.vstack([self.state_history, self.state])

        # Calculate Errors
        state_error = self.state - self.ideal_trajectory[current_step]
        self.error_history = np.vstack([self.error_history, state_error])

        # Call Controller
        self.K = self.compute_K(self.A, self.B)
        self.U = np.dot(-self.K, np.append(state_error, state_error[0:3])) + self.linearized_u
        self.clean_control_signal()
        
        self.u_history = np.vstack([self.u_history, np.dot(self.R, self.U)]) # Rotated into rocket frame
        
        # Convert desired accelerations to throttle and gimbal angles
        self.posx, self.posy, self.throttle = self.accelerations_2_actuator_positions()
        self.posx_history = np.append(self.posx_history, self.posx)
        self.posy_history = np.append(self.posy_history, self.posy)
        self.throttle_history = np.append(self.throttle_history, self.throttle)
            
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
        R = Rotation.from_euler('xyz', [yaw, -pitch, -roll]).as_matrix() # Cannot use self.R here because we are varying the state slightly for jacobian
        R_inv = np.linalg.inv(R) # Cannot use self.R_inv here because we have varied the state slightly for jacobian
        
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
        statedot[6:9] = get_EA_dot(state, self.R, R_inv)
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

    def compute_K(self, A, B):
        """Compute the K matrix for the flight phase of the mission using lqr
        """
                
        # Remove Roll Columns and Rows
        A = np.delete(A, 11, 0)
        A = np.delete(A, 11, 1)
        A = np.delete(A, 8, 0)
        A = np.delete(A, 8, 1)
        B = np.delete(B, 11, 0)
        B = np.delete(B, 8, 0)
                
        Q, R = self.get_QR_LQR()
                
        # Control
        C = np.append(np.identity(3), np.zeros((3, 7)), axis=1) # C is of the form y = Cx, where x is the state and y is the steady state error we care about - in this case just [x y z]
    
        K,S,E = control.lqr(A, B, Q, R, integral_action=C) # The K that this spits out has the additional 3 integral terms tacked onto the end, so must add errorx, y, z onto end of state error when solving for U
        K = np.insert(K, 8, 0, axis=1)
        K = np.insert(K, 11, 0, axis=1)
        
        return K
    
    def get_QR_LQR(self):
        # Q and R
        Q = np.identity(len(self.state) - 2 + 3) #Minus 2 to remove roll stuff plus 3 to get integral control
        R = np.identity(len(self.U))

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
    
    def update_linearization(self):
        """ Update linearization to account for rocket rotation
        
        Inputs:
        - Old A matrix
        - Old B matrix
        - Rocket rotation matrix R
        
        Output
        - rotated A 
        - rotated B
        
        """
    
        self.A[9:12,6:9] = np.dot(self.R, self.A[9:12,6:9])
        self.A[9:12,9:12] = np.dot(self.R, self.A[9:12,9:12])
        self.B[9:12,0:3] = np.dot(self.R, self.B[9:12,0:3])
        
    def clean_control_signal(self):
        if not self.t == 0:
            self.U[0] = self.low_pass_filter(self.U[0], self.u_history[-1][0], 1)
            self.U[1] = self.low_pass_filter(self.U[1], self.u_history[-1][1], 1)
            self.U[2] = self.low_pass_filter(self.U[2], self.u_history[-1][2], 0.75)
            
    def low_pass_filter(self, signal, previous_signal_filtered, alpha):
        filtered_value = alpha * signal + (1 - alpha) * previous_signal_filtered
        return filtered_value
    
    def kalman_filter(self):
        """ Kalman filter"""
        
        def _predict_step(x, u, A, B, P_prev, Q, dt):
            """ Prediction step for kalman filter"""
            
            # It is way too much work to make a state transition model by differentiating the dynamics so do this instead
            F = np.eye(12) + self.A*dt
            
            # If we are off course and requesting a larger throttle do not use the control input in kalman
            if self.get_throttle(self.mass * np.sqrt((u[0] ** 2) + (u[1] ** 2) + (u[2] ** 2))) > 1:
                u = np.array([0,0,0])
             
            x_next = F @ x #+ (B*dt) @ (u - self.linearized_u) # Transition State to next state using our understanding of dynamics.
            P_next = F @ P_prev @ F.T + Q # Transition estimate covariance to next step
            return x_next, P_next
        
        def _update_step(x_next, P_next, measurement, H, R):
            """ Update step for kalman filter"""
            
            # If a sensor has no measurement this timestep, use the expected value (z)
            z = H @ x_next
            for i in range(len(measurement)):
                if math.isnan(measurement[i]):
                    measurement[i] = z[i]

            # Kalman update steps
            y = measurement - z # measurement with truth - expected (using x_next)
            K = P_next @ H.T @ np.linalg.inv(H @ P_next @ H.T + R)
            x_fit = x_next + K @ y
            P_fit = (np.eye(12) - K @ H) @ P_next
            return x_fit, P_fit
        
        # Create Process Noise Matrix
        Q = np.eye(12)

        # H assumes sensor format [x y z xdot ydot zdot z xdot ydot zdot p y r pdot ydot rdot]
        H = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],       
        ]) 
    
        R = np.eye(16)
        R[0,0] = 1
        R[1,1] = 1
        R[2,2] = 1
        R[3,3] = 10
        R[4,4] = 10
        R[5,5] = 10
        R[6,6] = 200
        
        # Update Step
        x_next, P_next = _predict_step(self.state, self.U, self.A, self.B, self.kalman_P, Q, self.ts)
        
        # Measurement Step
        x_fit, P_fit = _update_step(x_next, P_next, self.measurement, H, R)

        self.state = x_fit
        self.kalman_P = P_fit
        
    def get_throttle(self, thrust):
        """ Takes in a query time and a thrust and outputs the throttle based on the thrust curve
        
        Inputs:
            t = requested time for query (with t=0 being engine startup)
            thrust = requested thrust
            
        Returns:
            throttle = throttle given conditions above
            
        """

        # Calculate "Real" Postition on thrust curve, based off of throttle history
        if len(self.throttle_history) == 0:
            t_adj = 0
        else:
            t_adj = np.sum(self.throttle_history) * self.ts

        # Calculate Thrust at given time
        if t_adj >= self.burn_duration:
            maxThrust = 0
        else:
            maxThrust = self.thrust_curve[int(t_adj/self.dt_thrust_curve)]
        self.thrust = maxThrust

        # Apply Throttling
        if maxThrust == 0:
            throttle = 1
        else:
            throttle = thrust / maxThrust

        return(throttle)
    
    def accelerations_2_actuator_positions(self):
        """ Convert control input to engine position and throttle
        
        Inputs:
        - Control input in global frame (1x3)
        - rocket object
        - current time
        
        Output:
        - pos_x, the x position of the engine
        - pos_y, the y position of the engine
        - throttle, the throttle percent of the engine
        """
        # Rotate into rocket frame
        U = np.dot(self.R, self.U) 
        gimbal_theta = np.arctan2(-U[1], -U[0])
        gimbal_psi = np.arctan2(np.sqrt((U[1] ** 2) + (U[0] ** 2)), U[2])
        T = self.mass * np.sqrt((U[0] ** 2) + (U[1] ** 2) + (U[2] ** 2))
        gimbal_r = np.tan(gimbal_psi) * self.engine_length
        if (gimbal_theta < np.pi / 2) and (gimbal_theta > -np.pi / 2):
            pos_x_commanded = np.sqrt((gimbal_r ** 2) / (1 + (np.tan(gimbal_theta) ** 2)))
        else:
            pos_x_commanded = -1 * np.sqrt((gimbal_r ** 2) / (1 + (np.tan(gimbal_theta) ** 2)))
        pos_y_commanded = pos_x_commanded * np.tan(gimbal_theta)
        throttle_commanded = self.get_throttle(T)
        
        return pos_x_commanded, pos_y_commanded, throttle_commanded