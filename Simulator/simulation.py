import numpy as np
import scipy.integrate
import Vehicle.engine
import Vehicle.rocket
import Control.throttle

# constants
g = 9.81

class Simulation:
    """ Class Representing the Rocket and associated data"""
    def __init__(self, timefinal, simulation_timestep, starting_state, planned_trajectory):
        # Create Engine Object inside Rocket
        self.state = starting_state
        self.rocket = Vehicle.rocket.Rocket(simulation_timestep)
        self.timestep = simulation_timestep
        self.timefinal = timefinal
        self.t_prev = 0
        self.ideal_trajectory = planned_trajectory
        self.errorHistory = np.empty((0))
        
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

        # Propogate given ODE
        solution = scipy.integrate.solve_ivp(self.wrapper_state_to_stateDot, t, state, args=(self.rocket, self.ideal_trajectory, t_span), t_eval=t_span, max_step=ts/5)
        return solution['y'].T

    def wrapper_state_to_stateDot(self, t, state, rocket, ideal_trajectory, t_vec):
        global previous_time
        global currStep
        if t == 0:
            currStep = 0

        # Check if we are on an actual simulation timestep or if this is ode solving shenanigans
        if (t == 0) or (t >= t_vec[currStep] and previous_time < t_vec[currStep]):

            # FIND ACTUATOR ADJUSTMENTS
            ideal_state = ideal_trajectory[currStep]
            error = ideal_state - state[0:3]

            # Save error to error history
            if t == 0:
                self.errorHistory = error.reshape((1, 3))
            else:
                self.errorHistory = np.append(self.errorHistory, error.reshape((1, 3)), axis=0)

            throttle = Control.throttle.getThrottle(rocket.engine.throttle, error)
            theta_x = 0.0
            theta_y = 0.0
            dt = t_vec[1] - t_vec[0]
            
            # Log Current States
            rocket.engine.save_throttle(throttle)
            rocket.engine.save_thetaX(theta_x)
            rocket.engine.save_thetaY(theta_y)
            rocket.engine.save_thrust(rocket.engine.get_thrust(t, throttle))
            rocket.update_mass(dt)

            if not t == t_vec[-1]:
                currStep += 1

        previous_time = t
        return self.state_to_stateDot(t, state, rocket)

    def state_to_stateDot(self, t, state, rocket):

        # Pull Params
        throttle = rocket.engine.throttle
        T = rocket.engine.get_thrust(t, throttle)
        m = rocket.mass
        thetaX = rocket.engine.thetax
        thetaY = rocket.engine.thetay

        # Build Statedot
        statedot = np.zeros(6)
        statedot[0:3] = state[3:6]

        # Calculate Acceleration
        statedot[3:6] = [T * np.sin(thetaX), T * np.sin(thetaY), (T * np.cos(thetaX) / m) - g]
        return statedot
    