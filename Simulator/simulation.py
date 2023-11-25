import numpy as np
import scipy.integrate
import Vehicle.engine
import Vehicle.rocket
from Control.controller import PIDController
import Control.controlConstants
from scipy.spatial.transform import Rotation
from Simulator.simulationConstants import GRAVITY as g
from Simulator.simulationConstants import RHO as rho

class Simulation:
    """ Class Representing the Rocket and associated data"""
    def __init__(self, timefinal, simulation_timestep, starting_state, wind, planned_trajectory):
        # Create Engine Object inside Rocket
        self.state = starting_state
        self.rocket = Vehicle.rocket.Rocket(simulation_timestep)
        self.timestep = simulation_timestep
        self.timefinal = timefinal
        self.previous_time = 0
        self.current_step = 0
        self.ideal_trajectory = planned_trajectory
        self.position_error_history = np.array([[0,0,0]]) 
        self.rotation_error_history = np.array([[0,0,0]]) 
        self.wind = wind

        #PID controller 
        self.throttle_controller = PIDController(kp=Control.controlConstants.KP_CONSTANT_THROTTLE, ki=Control.controlConstants.KI_CONSTANT_THROTTLE, kd=Control.controlConstants.KD_CONSTANT_THROTTLE)
        self.pos_x_controller = PIDController(kp=Control.controlConstants.KP_CONSTANT_POS, ki=Control.controlConstants.KI_CONSTANT_POS, kd=Control.controlConstants.KD_CONSTANT_POS)
        self.pos_y_controller = PIDController(kp=Control.controlConstants.KP_CONSTANT_POS, ki=Control.controlConstants.KI_CONSTANT_POS, kd=Control.controlConstants.KD_CONSTANT_POS)
        self.theta_y_controller = PIDController(kp=Control.controlConstants.KP_CONSTANT_THETA, ki=Control.controlConstants.KI_CONSTANT_THETA, kd=Control.controlConstants.KD_CONSTANT_THETA)
        self.theta_x_controller = PIDController(kp=Control.controlConstants.KP_CONSTANT_THETA, ki=Control.controlConstants.KI_CONSTANT_THETA, kd=Control.controlConstants.KD_CONSTANT_THETA)
        
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
            if t < 10 * ts:
                return 1
            else:
                return y[2]
        event.terminal=True
        event.direction=-1
        solution = scipy.integrate.solve_ivp(self.wrapper_state_to_stateDot, t, state, args=(self.rocket, self.ideal_trajectory, t_span), t_eval=t_span, max_step=ts/5, events=event)
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
            ideal_position_state = ideal_trajectory[self.current_step]
            ideal_rotational_state = [0, 0, 0]
            position_error = ideal_position_state - state[0:3]
            rotational_error = ideal_rotational_state - state[6:9]
            dt = t_vec[1] - t_vec[0]

            if t == 0:
                self.position_error_history = position_error.reshape((1, 3))
                self.rotation_error_history = rotational_error.reshape((1, 3))

            # Find Actuator Valuess
            # STEVE BRUNTON VIDEOS, generalized pid control with jacobian
            throttle = self.throttle_controller.control(position_error[2], self.position_error_history[-1][2], dt, 'throttle')
            # Check if we have angular velocity. Correct to verticle if so. Else, adjust position
            if (abs(state[6]) > self.rocket.tip_angle): 
                pos_x = self.theta_x_controller.control(rotational_error[0], self.rotation_error_history[-1][0], dt, 'posx')
            else:
                pos_x = self.pos_x_controller.control(position_error[0], self.position_error_history[-1][0], dt, 'posx')
            if (abs(state[7]) > self.rocket.tip_angle):
                pos_y = self.theta_y_controller.control(rotational_error[1], self.rotation_error_history[-1][1], dt, 'posy')
            else:
                pos_y = self.pos_y_controller.control(position_error[1], self.position_error_history[-1][1], dt, 'posy')

            # Save error to error history
            if not t == 0:
                self.position_error_history = np.append(self.position_error_history, position_error.reshape((1, 3)), axis=0)
                self.rotation_error_history = np.append(self.rotation_error_history, rotational_error.reshape((1, 3)), axis=0)

            # Log Current States
            rocket.engine.save_throttle(throttle)
            rocket.engine.save_posX(pos_x)
            rocket.engine.save_posY(pos_y)
            rocket.engine.save_thrust(rocket.engine.get_thrust(t, throttle))
            rocket.update_mass(dt)
            rocket.update_I()

            if not t == t_vec[-1]:
                self.current_step += 1
            self.previous_time = t
        
        return self.state_to_stateDot(t, state, rocket)

    def state_to_stateDot(self, t, state, rocket):

        # Pull Params
        throttle = rocket.engine.throttle
        T = rocket.engine.get_thrust(t, throttle)
        m = rocket.mass
        lever_arm = rocket.com
        engine_length = rocket.engine.length
        posX = rocket.engine.posx
        posY = rocket.engine.posy
        v = state[3:6]
        w = state[9:12]

        # Convert Actuator Positions to Cyclindrical Coords
        gimbal_R = np.sqrt((posX ** 2) + (posY ** 2))
        gimbal_theta = np.arctan2(posY, posX)
        gimbal_psi = np.arctan2(gimbal_R, engine_length)

        # Build Statedot
        statedot = np.zeros(len(state))
        statedot[0:3] = v
        statedot[6:9] = w
        
        # Rocket rotations
        pitch = state[6] # Angle from rocket from pointing up towards positive x axis
        yaw = state[7] # Angle from rocket from pointing up towards positive y axis
        roll = state[8] # Roll, ccw when looking down on rocket
        R = Rotation.from_euler('xyz', [yaw, -pitch, -roll]).as_matrix()
        R_inv = np.linalg.inv(R)

        # Wind rotation into rocket frame
        wind_rf = np.dot(R, self.wind + v)
        wind_force = rocket.find_wind_force(wind_rf, rho)
        wind_moment = rocket.find_wind_moment(wind_rf, rho)

        # Calculate Accelerations in rocket frame
        aX_rf = (T * np.sin(gimbal_psi) * np.cos(gimbal_theta) / m) + (-1 * g * R[0][2]) + (wind_force[0] / m)
        aY_rf = (T * np.sin(gimbal_psi) * np.sin(gimbal_theta) / m) + (-1 * g * R[1][2]) + (wind_force[1] / m)
        aZ_rf = (T * np.cos(gimbal_psi) / m) + (-1 * g * R[2][2]) + (wind_force[2] / m)
        a_rf = np.array([aX_rf, aY_rf, aZ_rf])

        # Convert Accelerations from rocket frame to global frame
        a_global = np.dot(R_inv, a_rf)

        # Calculate Alphas
        torque = np.array([(T * np.sin(gimbal_psi) * np.cos(gimbal_theta) * lever_arm) + wind_moment[0],
                          (T * np.sin(gimbal_psi) * np.sin(gimbal_theta) * lever_arm) + wind_moment[1],
                          0])
        I_dot = rocket.get_I_previous()
        alphas = np.dot(rocket.I_inv, torque - np.cross(w, np.dot(rocket.I, w)) - np.dot(I_dot, w))

        statedot[3:6] = a_global.tolist()
        statedot[9:12] = alphas.tolist()

        return statedot
    
