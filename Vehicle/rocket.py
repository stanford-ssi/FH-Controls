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
        
        # Cg and Cp locations, measured from top of rocket
        self.com = 0.75
        self.cop = Vehicle.rocketConstants.Cp

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

    def find_wind_force(self, wind, rho):
        """ Outputs the expected wind force vector [fx, fy, fz]
        
        Inputs:
            Wind in the rocket frame
            rho density of air
            
        Returns:
            force - wind force vector in rf
            
        """
        alpha = np.arccos(np.dot(wind, [0,0,1]) / np.linalg.norm(wind))
        Cn = Vehicle.rocketConstants.Cna * alpha
        Aref = Vehicle.rocketConstants.AREF
        Fn = 0.5 * Cn * rho * (np.linalg.norm(wind) ** 2) * Aref #Normal Force on Rocket
        Fx = Fn * wind[0] / ((np.sqrt((wind[0] ** 2) + (wind[1] ** 2))))
        Fy = Fn * wind[1] / ((np.sqrt((wind[0] ** 2) + (wind[1] ** 2))))
        Fz = 0.5 * Vehicle.rocketConstants.Cd * rho * (wind[2] ** 2) * np.pi * ((Vehicle.rocketConstants.ROCKET_DIAMETER / 2) ** 2)
        Force = [Fx, Fy, Fz]
        return Force

    def find_wind_moment(self, wind, rho):
        """ Outputs the expected wind moment [Mx, My, Mz]. Assumes symmetric rocket so 0 moment in roll. Can update later.
        
        Inputs:
            Wind in the rocket frame
            rho density of air
            
        Returns:
            force - wind moment vector in rf
            
        """

        force = self.find_wind_force(wind, rho)[0:2]
        lever_arm = self.com - self.cop
        Mx = force[0] * lever_arm
        My = force[1] * lever_arm
        moment = [Mx, My, 0]
        return moment