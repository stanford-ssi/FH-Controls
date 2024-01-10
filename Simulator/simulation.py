import numpy as np
import scipy.integrate
import Vehicle.engine
import Vehicle.rocket
import control
from GNC.constraints import *
import GNC.controlConstants
from scipy.spatial.transform import Rotation
from Simulator.dynamics import *
from GNC.math import compute_A, compute_B
from Simulator.simulationConstants import GRAVITY as g
from Simulator.simulationConstants import RHO as rho

class Simulation:
    """ Class Representing the Rocket and associated data"""
    def __init__(self, timefinal, simulation_timestep, starting_state, wind, planned_trajectory):
        # Create Engine Object inside Rocket
        self.state_previous = starting_state
        self.state = starting_state
        self.statedot_previous = np.zeros((1,len(self.state)))
        self.rocket = Vehicle.rocket.Rocket(simulation_timestep)
        self.timestep = simulation_timestep
        self.timefinal = timefinal
        self.previous_time = 0
        self.current_step = 0
        self.ideal_trajectory = planned_trajectory
        self.position_error_history = np.array([[0,0,0]]) 
        self.rotation_error_history = np.array([[0,0,0]]) 
        self.wind = wind

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
        tf = self.timefinal
        ts = self.timestep

        state = self.state

        # Create t vector from 0 to tf with timestep ts
        t_span = np.linspace(0, int(tf/ts)*ts, num=int(tf/ts)+1)
        t = (0,self.timefinal)

        # Propogate given ODE, stop when rocket crashes as indicated by this here event function
        def event(t,y,r,it,tt):
            if t < 100 * ts:
                return 1
            else:
                return y[2]
        event.terminal=True
        event.direction=-1
        solution = scipy.integrate.solve_ivp(self.wrapper_state_to_stateDot, t, state, args=(self.rocket, self.ideal_trajectory, t_span), t_eval=t_span, max_step=ts/5, events=event)
        print(self.jacobian_error)
        return solution['y'].T

    def display_end_info(self):
        print()
        print("Simulation Ended at t = ", self.previous_time, "s, at simulation step ", self.current_step)
        print()
        if self.previous_time == self.timefinal:
            print("Sucsessful Execution of Planned Trajectory")
        else:
            print("Unsucsessful Execution of Planned Trajectory")
            print("VEHICLE CRASH DETECTED")
        print()
        print("Rocket Start Mass: ", self.rocket.massHistory[0], "kg | End Mass: ", self.rocket.mass)
        print("Engine Start Mass: ", self.rocket.engine.full_mass, "kg | End Mass: ", self.rocket.engine.mass)
        print("Percent Fuel Remaining: ", 1 - (self.rocket.engine.full_mass - self.rocket.engine.mass) / (self.rocket.engine.full_mass - self.rocket.engine.drymass))
        print()

    def wrapper_state_to_stateDot(self, t, state, rocket, ideal_trajectory, t_vec):

        # Check if we are on an actual simulation timestep or if this is ode solving shenanigans
        if (t == 0) or (t >= t_vec[self.current_step] and self.previous_time < t_vec[self.current_step]):

            # Calculate Errors
            ideal_position_state = np.concatenate((ideal_trajectory[self.current_step], np.array([0,0,0])), axis=0)
            ideal_rotational_state = [0, 0, 0, 0, 0, 0]
            position_error = ideal_position_state - state[0:6]
            rotational_error = ideal_rotational_state - state[6:12]
            state_error = np.concatenate((position_error, rotational_error), axis=0)

            if t == 0:
                self.position_error_history = position_error.reshape((1, 6))
                self.rotation_error_history = rotational_error.reshape((1, 6))

            # State Space Control
            linearized_x = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
            linearized_u = np.array([0, 0, g])
            A_orig = compute_A(linearized_x, linearized_u, rocket, self.wind, self.timestep)
            B_orig = compute_B(linearized_x, linearized_u, rocket, self.timestep)
            
            # Remove Roll Columns and Rows
            A = np.delete(A_orig, 11, 0)
            A = np.delete(A, 11, 1)
            A = np.delete(A, 8, 0)
            A = np.delete(A, 8, 1)
            B = np.delete(B_orig, 11, 0)
            B = np.delete(B, 8, 0)
            Q = np.identity(len(state) - 2)
            R = np.identity(len(linearized_u))
            K,S,E = control.lqr(A, B, Q, R)
            K = np.insert(K, 8, 0, axis=1)
            K = np.insert(K, 11, 0, axis=1)
            U = np.dot(-K, state_error) + linearized_u # U is the desired accelerations
            
            # Save error to error history
            if not t == 0:
                self.position_error_history = np.append(self.position_error_history, position_error.reshape((1, 6)), axis=0)
                self.rotation_error_history = np.append(self.rotation_error_history, rotational_error.reshape((1, 6)), axis=0)

            # Convert desired accelerations to throttle and gimbal angles
            gimbal_theta = np.arctan2(U[1], U[0])
            gimbal_psi = np.arctan2(U[0], U[2] * np.cos(gimbal_theta))
            T = U[2] * rocket.mass / np.cos(gimbal_psi)
            gimbal_r = np.tan(gimbal_psi) * rocket.engine.length
            if gimbal_theta > 0:
                pos_x = np.sqrt((gimbal_r ** 2) / (1 + (np.tan(gimbal_theta) ** 2)))
            else:
                pos_x = -1 * np.sqrt((gimbal_r ** 2) / (1 + (np.tan(gimbal_theta) ** 2)))
            pos_y = pos_x * np.tan(gimbal_theta)
            throttle = rocket.engine.get_throttle(t, T)
            
            # Check if railed
            throttle = throttle_checks(throttle)
            # pos_x = pos_checks(pos_x)
            # pos_y = pos_checks(pos_y)

            # Log Current States
            rocket.engine.save_throttle(throttle)
            rocket.engine.save_posX(pos_x)
            rocket.engine.save_posY(pos_y)
            rocket.engine.save_thrust(rocket.engine.get_thrust(t, throttle))
            rocket.update_rocket()

            if not t == t_vec[-1]:
                self.current_step += 1
            self.previous_time = t
        
        if t == 0:
            self.jacobian_error = 0
            self.statedot_previous = natural_dyanamics(state, rocket, self.wind, self.timestep)
        
        statedot = full_dynamics(state, rocket, self.wind, self.timestep, t)
        return statedot