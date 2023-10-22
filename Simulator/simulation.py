import numpy as np
import scipy.integrate
import Simulator.ode
import Vehicle.rocket

class Simulation:
    """ Class Representing the Rocket and associated data"""
    def __init__(self, timefinal, simulation_timestep, starting_state):
        # Create Engine Object inside Rocket
        self.state = starting_state
        self.rocket = Vehicle.rocket.Rocket(simulation_timestep)
        self.timestep = simulation_timestep
        self.timefinal = timefinal
        
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
        t = np.linspace(0, int(tf/ts)*ts, num=int(tf/ts))

        # Propogate given ODE
        solution = scipy.integrate.odeint(Simulator.ode.state_to_stateDot, state, t, args=(self.rocket,))
        return solution