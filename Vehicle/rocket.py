import numpy as np
import Vehicle.engine
import Vehicle.rocketConstants
from scipy.integrate import cumtrapz
from scipy.interpolate import interp1d

class Rocket:
    """ Class Representing the Rocket and associated data"""
    def __init__(self, simulation_timestep):
        # Create Engine Object inside Rocket
        self.engine = Vehicle.engine.Engine(simulation_timestep)
        self.mass_noEngine = Vehicle.rocketConstants.ROCKET_MASS
        self.mass = self.mass_noEngine + self.engine.fullMass #Rocket Starts Fully Fueled
        self.massHistory = np.empty(shape=(0))
        
    def update_mass(self, t):
        """ Outputs the expected mass based on it
        
        Inputs:
            throttleHistory = history of throttle
            
        Returns:
            mass = mass of rocket at current query time
            
        """
        # Pull Data
        throttle_history = self.engine.throttleHistory
        thrust_curve = self.engine.thrustCurve      

        # Define a scaling constant for my shitty T~mdot assumption and calculate
        if len(throttle_history) > 1:
            constant = 0.0001
            self.engine.mass -= cumtrapz(constant * throttle_history * self.engine.thrustHistory)[0] * t
            self.mass = self.mass_noEngine + self.engine.mass
        self.massHistory = np.append(self.massHistory, self.mass)