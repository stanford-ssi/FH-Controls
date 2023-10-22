import numpy as np
import Vehicle.engine
import Vehicle.rocketConstants
from scipy.integrate import cumtrapz

class Rocket:
    """ Class Representing the Rocket and associated data"""
    def __init__(self, simulation_timestep):
        # Create Engine Object inside Rocket
        self.engine = Vehicle.engine.Engine(simulation_timestep)
        self.mass_noEngine = Vehicle.rocketConstants.ROCKET_MASS
        self.mass = self.mass_noEngine + self.engine.fullMass #Rocket Starts Fully Fueled
        
    def update_mass(self):
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
        constant = 0.001
        self.engine.mass -= cumtrapz(constant * throttle_history * thrust_curve[:len(throttle_history)])[0]
        self.mass = self.mass_noEngine + self.engine.mass