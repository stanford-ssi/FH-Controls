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
        
        self.com = 1.25
        self.I_prev = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 5]])
        self.I = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 5]])
        self.I_inv = np.linalg.inv(self.I)

        self.tip_angle = Vehicle.rocketConstants.ROCKET_MAX_TIP
        self.shape = Vehicle.rocketConstants.ROCKET_SHAPE
        
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
        force = np.zeros((3))
        for i in range(len(self.shape)):
            force[0] += 0.5 * rho * (wind[0] ** 2) * self.shape[i][1] * self.shape[i][2] * (wind[0]/abs(wind[0]))
            force[1] += 0.5 * rho * (wind[1] ** 2) * self.shape[i][0] * self.shape[i][2] * (wind[1]/abs(wind[1]))
            force[2] += 0.5 * rho * (wind[2] ** 2) * self.shape[i][0] * self.shape[i][1] * (wind[2]/abs(wind[2]))
        return force

    def find_wind_moment(self, wind, rho):
        """ Outputs the expected wind moment [Mx, My, Mz]. Assumes symmetric rocket so 0 moment in roll. Can update later.
        
        Inputs:
            Wind in the rocket frame
            rho density of air
            
        Returns:
            force - wind moment vector in rf
            
        """

        force = self.find_wind_force(wind, rho)
        sum = np.zeros(3)
        component_start_pos = 0
        total_area = np.zeros(3)

        # sum area times distance from bottom of rocket to find cop for each side
        for i in reversed(range(len(self.shape))):
            sum[0] += (self.shape[i][0] * self.shape[i][2]) * (component_start_pos + (self.shape[i][2] / 2))
            sum[1] += (self.shape[i][1] * self.shape[i][2]) * (component_start_pos + (self.shape[i][2] / 2))
            component_start_pos += self.shape[i][2]
            total_area[0] += self.shape[i][0] * self.shape[i][2]
            total_area[1] += self.shape[i][1] * self.shape[i][2]
            total_area[2] += self.shape[i][0] * self.shape[i][1]
        
        # Center of Pressure Calc for each side
        cop = np.divide(sum, total_area)    

        # Moment from wind force times distance between cg and cp
        moment = [force[i] * (cop[i] - self.com) for i in range(len(force) - 1)]
        return moment