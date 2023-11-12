import numpy as np
import scipy.integrate
import Vehicle.engine
import Vehicle.rocket
from Control.controller import PIDController
import Simulator.math
import Control.controlConstants


# constants (SHOULD WE HAVE THIS IN A SEPARATE FILE LIKE OTHER CONSTANTS?)
g = 9.81

class Simulation:
    """ Class Representing the Rocket and associated data"""
    def __init__(self, timefinal, simulation_timestep, starting_state, planned_trajectory):
        # Create Engine Object inside Rocket
        self.state = starting_state
        self.rocket = Vehicle.rocket.Rocket(simulation_timestep)
        self.timestep = simulation_timestep
        self.timefinal = timefinal
        self.previous_time = 0
        self.current_step = 0
        self.ideal_trajectory = planned_trajectory
        self.position_error_history = np.empty((0)) 
        self.rotation_error_history = np.empty((0)) 

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
            position_error = state[0:3] - ideal_position_state
            rotational_error = state[6:9] - ideal_rotational_state
            dt = t_vec[1] - t_vec[0]

            # Save error to error history
            if t == 0:
                self.position_error_history = position_error.reshape((1, 3))
                self.rotation_error_history = position_error.reshape((1, 3))
            else:
                self.position_error_history = np.append(self.position_error_history, position_error.reshape((1, 3)), axis=0)
                self.rotation_error_history = np.append(self.rotation_error_history, rotational_error.reshape((1, 3)), axis=0)
    
            #Find Actuator Values
            throttle = 0.5#self.throttle_controller.control(-1 * position_error[2], dt, 'throttle')
            # Check if we have angular velocity. Correct to verticle if so. Else, adjust position
            if (state[9] > 0.001): 
                pos_x = self.theta_x_controller.control(-1 * rotational_error[0], dt, 'posx')
            else:
                pos_x = self.pos_x_controller.control(-1 * position_error[0], dt, 'posx')
            if (state[10] > 0.001):
                pos_y = self.theta_y_controller.control(-1 * rotational_error[1], dt, 'posy')
            else:
                pos_y = self.pos_y_controller.control(-1 * position_error[1], dt, 'posy')
            
            pos_x = self.theta_x_controller.control(-1 * rotational_error[0], dt, 'posx')
            pos_y = self.theta_y_controller.control(-1 * rotational_error[1], dt, 'posy')

            # Log Current States
            rocket.engine.save_throttle(throttle)
            rocket.engine.save_posX(pos_x)
            rocket.engine.save_posY(pos_y)
            rocket.engine.save_thrust(rocket.engine.get_thrust(t, throttle))
            rocket.update_mass(dt)

            if not t == t_vec[-1]:
                self.current_step += 1
            self.previous_time = t
        
        return self.state_to_stateDot(t, state, rocket)

    def state_to_stateDot(self, t, state, rocket):

        # Pull Params
        throttle = rocket.engine.throttle
        T = rocket.engine.get_thrust(t, throttle)
        m = rocket.mass
        lever_arm = rocket.lever_arm
        engine_length = rocket.engine.length
        posX = rocket.engine.posx
        posY = rocket.engine.posy

        # Convert Actuator Positions to Cyclindrical Coords
        gimbal_R = np.sqrt((posX ** 2) + (posY ** 2))
        gimbal_theta = np.arctan2(posY, posX)
        gimbal_psi = np.arctan2(gimbal_R, engine_length)


        # Build Statedot
        statedot = np.zeros(len(state))

        # ROTATE GLOBAL STATE INTO ROCKETFRAME

        statedot[0:3] = state[3:6]
        statedot[6:9] = state[9:12]
        
        # Rocket rotations
        pitch = state[6]
        yaw = state[7]
        roll = state[8]
        R = Simulator.math.euler_matrix(yaw, pitch, roll)

        # Calculate Accelerations in rocket frame
        aX = (T * np.sin(gimbal_psi) * np.cos(gimbal_theta) / m) + (-1 * g * R[0][2])
        aY = (T * np.sin(gimbal_psi) * np.sin(gimbal_theta) / m) + (-1 * g * R[1][2])
        aZ = (T * np.cos(gimbal_psi) / m) + (-1 * g * R[2][2])

        # Calculate Alphas
        torque = [T * np.sin(gimbal_psi) * np.cos(gimbal_theta) * lever_arm,
                  T * np.sin(gimbal_psi) * np.sin(gimbal_theta) * lever_arm,
                  0]
        alphax =  torque[0] * rocket.I_inv[0,0] + torque[1] * rocket.I_inv[0,1] + torque[2] * rocket.I_inv[0,2]
        alphay =  torque[0] * rocket.I_inv[1,0] + torque[1] * rocket.I_inv[1,1] + torque[2] * rocket.I_inv[1,2]
        alphaz =  torque[0] * rocket.I_inv[2,0] + torque[1] * rocket.I_inv[2,1] + torque[2] * rocket.I_inv[2,2]

        statedot[3:6] = [aX, aY, aZ]
        statedot[9:12] = [alphax, alphay, alphaz]

        # ROTATE ROCKETFRAME STATEDOT INTO GLOBAL FRAME
        return statedot
    
