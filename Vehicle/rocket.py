import numpy as np
import Vehicle.engine
import Vehicle.rocketConstants
from scipy.integrate import cumtrapz
from scipy.interpolate import interp1d

class Rocket:
    """ Class Representing the Rocket and associated data"""
    def __init__(self, simulation_timestep):
        # Create Engine Object inside Rocket
        self.dt = simulation_timestep
        self.engine = Vehicle.engine.Engine(simulation_timestep)
        self.mass_noEngine = Vehicle.rocketConstants.ROCKET_MASS
        self.mass = self.mass_noEngine + self.engine.full_mass #Rocket Starts Fully Fueled
        self.massHistory = np.empty(shape=(0))
        
        self.lever_arm = 0.5
        self.I_prev = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 5]])
        self.I = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 5]])
        self.I_inv = np.linalg.inv(self.I)

        self.tip_angle = Vehicle.rocketConstants.ROCKET_MAX_TIP
        
    def update_mass(self, dt):
        """ Outputs the expected mass based on it
        
        Inputs:
            throttleHistory = history of throttle
            
        Returns:
            mass = mass of rocket at current query time
            
        """
        self.engine.mass -= (self.engine.thrust *  self.engine.throttle / self.engine.exhaust_velocity) * dt
        self.mass = self.mass_noEngine + self.engine.mass
        self.massHistory = np.append(self.massHistory, self.mass)

    def get_I_previous(self):
        return (self.I - self.I_prev) / self.dt
    
    def update_I(self):
        pass