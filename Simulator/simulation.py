import numpy as np
import scipy.integrate
import Vehicle.engine
import Vehicle.rocket
from Vehicle.sensors import *
from GNC.constraints import *
from GNC.controller import *
from GNC.kalmanFilter import *
from scipy.spatial.transform import Rotation
from Simulator.dynamics import *
from Simulator.errorInjection import *
from Simulator.simulationConstants import GRAVITY as g
from Simulator.simulationConstants import RHO as rho
from scipy.spatial.transform import Rotation

class Simulation:
    """ Class Representing the Simulation and associated data"""
    def __init__(self, timefinal, simulation_timestep, starting_state, wind, planned_trajectory):
        # Create Engine Object inside Rocket
        self.state_previous = starting_state
        self.state = roll_injection(starting_state)
        self.statedot_previous = np.zeros(len(self.state))
        self.rocket = Vehicle.rocket.Rocket(simulation_timestep)
        self.ideal_trajectory = planned_trajectory
        self.position_error_history = np.array([[0,0,0]]) 
        self.rotation_error_history = np.array([[0,0,0]])
        self.u = np.array([[0,0,0]])
        
        # Simulation Variables
        self.ts = simulation_timestep
        self.tf = timefinal
        self.previous_time = 0
        self.current_step = 0
        self.landed = False
        self.landing_violation = None
        
        # Initialize situation
        self.wind_history = np.array([[0,0,0]]) 
        self.base_wind = np.array([np.random.normal(0, wind[0]), np.random.normal(0, wind[1]), np.random.normal(0, wind[2])])
        self.current_wind = self.base_wind
        
        # Preform initial controller calculations
        self.linearized_x = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
        self.linearized_u = np.array([0, 0, g])
        self.A_orig = compute_A(self.linearized_x, self.linearized_u, self.rocket, self.ts)
        self.B_orig = compute_B(self.linearized_x, self.linearized_u, self.rocket, self.ts)
        
        # Sensors:
        self.sensed_state = np.array([[0,0,0,0,0,0,0,0,0,0,0,0]])
        self.kalman_state = np.array([[0,0,0,0,0,0,0,0,0,0,0,0]])
        self.kalman_P = np.eye(12)

    def propogate(self):
        """ Simple propogator

        Inputs:
            state_0 = initial state
            tf = end time of simulation
            ts = timestep

        Returns:
            solution = position data at each timestamp
            
        """
        # Pull Time Information
        tf = self.tf
        ts = self.ts

        state = self.state

        # Create t vector from 0 to tf with timestep ts
        t_span = np.linspace(0, int(tf/ts)*ts, num=int(tf/ts)+1)
        t = (0,self.tf)

        # Propogate given ODE, stop when rocket crashes as indicated by this here event function
        def event(t,y,r,it,tt):
            if t < 100 * ts:
                return 1
            else:
                if self.landed == True:
                    return 0
                else:
                    return y[2]
        event.terminal=True
        event.direction=-1
        solution = scipy.integrate.solve_ivp(self.wrapper_state_to_stateDot, t, state, args=(self.rocket, self.ideal_trajectory, t_span), t_eval=t_span, max_step=ts/5, events=event)
        state_history = solution['y'].T
        return state_history

    def display_end_info(self):
        """ Display ending info about the simulation into the terminal"""
        
        print()
        print("Simulation Ended at t = ", self.previous_time, "s, at simulation step ", self.current_step)
        print()
        if self.landed:
            print("Sucsessful Execution of Planned Trajectory and Landing")
        else:
            print("VEHICLE CRASH DETECTED")
            print("Landing Condition Violation: ", self.landing_violation)
        print()
        print("Rocket Start Mass: ", self.rocket.massHistory[0], "kg | End Mass: ", self.rocket.mass)
        print("Engine Start Mass: ", self.rocket.engine.full_mass, "kg | End Mass: ", self.rocket.engine.mass)
        print("Percent Fuel Remaining: ", (1 - (self.rocket.engine.full_mass - self.rocket.engine.mass) / (self.rocket.engine.full_mass - self.rocket.engine.drymass)) * 100, "%")
        print()
        print("State: ", self.state)
        print()

    def check_landing(self, state, t):
        # Check if below threshold altitude
        if state[2] < self.rocket.engine.length + 0.1 and t > 0.75 * self.tf:
            # Check Z speed
            if abs(state[5]) < MAX_Z_SPEED:
                # Check XY speed
                if abs(state[4]) < MAX_XY_SPEED and abs(state[3]) < MAX_XY_SPEED:
                    # Check rotation
                    if abs(state[6]) < MAX_ROTATION and abs(state[7]) < MAX_ROTATION: 
                
                        if abs(state[9]) < MAX_ROTATION_SPEED and abs(state[10]) < MAX_ROTATION_SPEED:
                            self.landed = True
                            self.state = state
                        else:
                            self.landing_violation = "EXCESSIVE ROTATIONAL VELOCITY - {} rad/s or {} rad/s out of {} rad/s allowed".format(abs(state[9]), abs(state[10]), MAX_ROTATION_SPEED)
                    else:
                        self.landing_violation = "EXCESSIVE TILT - {} rad or {} rad out of {} rad allowed".format(abs(state[6]), abs(state[7]), MAX_ROTATION)
                else:
                    self.landing_violation = "EXCESSIVE LATERAL SPEED - {} m/s or {} m/s out of {} m/s allowed".format(abs(state[3]), abs(state[4]), MAX_XY_SPEED) 
            else:
                self.landing_violation = "EXCESSIVE DESCENT SPEED - {} m/s out of {} m/s allowed".format(abs(state[5]), MAX_Z_SPEED)
        else:
            self.landing_violation = "OFF COURSE"
                               
    def wrapper_state_to_stateDot(self, t, state, rocket, ideal_trajectory, t_vec):
        """ Wrapper for the dynamics, most of the work done in this step. It calls the controller and updates the rocket's state based on
        control inputs and fuel drain."""
        # Check if we are on an actual simulation timestep or if this is ode solving shenanigans
        if (t == 0) or (t >= t_vec[self.current_step] and self.previous_time < t_vec[self.current_step]):
            
            # Log Rocket Rotation                  
            rocket.R = Rotation.from_euler('xyz', [state[7], -state[6], -state[8]]).as_matrix()                  
              
            # Determine wind at this moment in time
            self.current_wind = wind_randomness(self.base_wind, self.current_wind)
            if t == 0:
                self.wind_history = np.array([self.current_wind])
            else:
                self.wind_history = np.append(self.wind_history, [self.current_wind], axis=0)
            
            # Pre Control Work - Rotate State Matrices into current frame                 
            A, B = update_linearization(self.A_orig, self.B_orig, rocket.R) 
            if (t / self.tf) < 0.8:
                self.K = compute_K_flight(len(self.state), A, B)
            else:
                self.K = compute_K_landing(len(self.state), A, B)

            # Sense the state from sensors
            sensed_state = np.concatenate((rocket.gps.reading(state), 
                                            rocket.accelerometer.read_velocity(state, self.statedot_previous[3:6]), 
                                            rocket.magnetometer.reading(state), 
                                            rocket.gyroscope.read_velocity(state, self.statedot_previous[9:12]))).reshape((1, 12))
            self.sensed_state = np.append(self.sensed_state, sensed_state, axis=0)
            kalman_state, self.kalman_P = kalman_filter(self.kalman_state[-1], self.statedot_previous[3:6], self.sensed_state[-1], self.A_orig, self.B_orig, self.ts, rocket.engine.length, P=self.kalman_P)
            self.kalman_state = np.append(self.kalman_state, kalman_state[None, :], axis=0)
            positional_state = state[0:6]
            rotational_state = state[6:12]
            # positional_state = kalman_state[0:6]
            # rotational_state = kalman_state[6:12]
            
            # Calculate Errors
            position_error = positional_state - ideal_trajectory[self.current_step]
            rotational_error = rotational_state - [0, 0, 0, 0, 0, 0]
            state_error = np.concatenate((position_error, rotational_error), axis=0)
            if t == 0:
                self.position_error_history = position_error.reshape((1, 6))
                self.rotation_error_history = rotational_error.reshape((1, 6))
            else:
                self.position_error_history = np.append(self.position_error_history, position_error.reshape((1, 6)), axis=0)
                self.rotation_error_history = np.append(self.rotation_error_history, rotational_error.reshape((1, 6)), axis=0)        

            # Call Controller
            U = control_rocket(self.K, state_error, self.linearized_u)
            
            # Convert desired accelerations to throttle and gimbal angles
            pos_x, pos_y, throttle = accelerations_2_actuator_positions(U, rocket, t)
            
            # Inject Error to actuator positions
            pos_x, pos_y, throttle = actuator_error_injection(pos_x, pos_y, throttle)
                        
            # Perform actuator constraint checks
            if not t == 0:
                throttle = throttle_checks(throttle, rocket.engine.throttle_history[-1], self.ts)
                pos_x = pos_checks(pos_x, rocket.engine.posx_history[-1], self.ts)
                pos_y = pos_checks(pos_y, rocket.engine.posy_history[-1], self.ts)
            
            # Log Current States
            rocket.engine.save_throttle(throttle)
            rocket.engine.save_posX(pos_x)
            rocket.engine.save_posY(pos_y)
            rocket.engine.save_thrust(rocket.engine.get_thrust(t, throttle))
            rocket.update_rocket()

            ################ END LOOP ON ROCKET ##################

            if not t == t_vec[-1]:
                self.current_step += 1
            self.previous_time = t
            
            # Check for landing
            self.check_landing(state, t)
        
        if t == 0:
            self.jacobian_error = 0
            self.statedot_previous = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        
        self.statedot_previous = full_dynamics(state, rocket, self.current_wind, self.ts, t)
        return self.statedot_previous