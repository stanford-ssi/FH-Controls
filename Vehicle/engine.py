import numpy as np
import Vehicle.engineConstants
from scipy.interpolate import interp1d

class Engine:
    """ Class Representing the Engine and associated data"""
    def __init__(self, simulation_timestep):
        # For now, Constant thrust curve determined here, will later be a file from prop that will be imported
        self.dt_thrust_curve = Vehicle.engineConstants.DT_THRUST_CURVE
        self.timestep = simulation_timestep
        self.burn_duration = Vehicle.engineConstants.ENGINE_BURN_DURATION
        self.exhaust_velocity = Vehicle.engineConstants.EXHAUST_VELOCITY
        self.thrust_curve = Vehicle.engineConstants.THRUST_CURVE
        self.thrust_history = np.empty(shape=(0)) # The thrust curve we will fill up for integration (Just keep it this way it's easier than trying to reshape the origional thrust curve every time)
        self.thrust = self.thrust_curve[0]

        # Initialize empty list to save throttle history in, to be used for calculating mass of rocket at a given time
        self.throttle_history = np.empty(shape=(0))
        self.throttle = 1
        self.posx_history = np.empty(shape=(0))
        self.posx = 0
        self.posy_history = np.empty(shape=(0))
        self.posy = 0
        self.gimbal_psi_history = np.empty(shape=(0))
        self.gimbal_psi = 0
        self.gimbal_theta_history = np.empty(shape=(0))
        self.gimbal_theta = 0
        self.gimbal_alpha = np.array([0,0,0])
        self.gimbal_alpha_history = np.empty(shape=(0))

        # Masses
        self.drymass = Vehicle.engineConstants.ENGINE_DRYMASS
        self.full_mass = Vehicle.engineConstants.ENGINE_FULL_MASS
        self.mass = self.full_mass
        self.length = Vehicle.engineConstants.LENGTH

    def get_thrust(self, t, throttle):
        """ Takes in a query time and a throttle percentage and outputs the thrust based on the thrust curve
        
        Inputs:
            t = requested time for query (with t=0 being engine startup)
            throttle = requested throttle percentage
            
        Returns:
            Thrust = Thrust given conditions above
            
        """

        # Calculate "Real" Postition on thrust curve, based off of throttle history
        if len(self.throttle_history) == 0:
            t_adj = 0
        else:
            t_adj = np.sum(self.throttle_history) * self.timestep
            
        # Calculate Thrust at given time
        if t_adj >= self.burn_duration:
            maxThrust = 0
        else:
            maxThrust = self.thrust_curve[int(t_adj/self.dt_thrust_curve)]
        self.thrust = maxThrust

        # Apply Throttling
        Thrust = maxThrust * throttle

        return(Thrust)
    
    def save_throttle(self, throttle):
        """ Takes in the current Throttle and saves it into the throttle history"""
        self.throttle_history = np.append(self.throttle_history, throttle)
        self.throttle = throttle

    def save_engine_positions(self):
        """ Takes in the current Throttle and saves it into the throttle history"""
        self.posx_history = np.append(self.posx_history, self.posx)
        self.posy_history = np.append(self.posy_history, self.posy)
        self.gimbal_psi_history = np.append(self.gimbal_psi_history, self.gimbal_psi)
        self.gimbal_theta_history = np.append(self.gimbal_theta_history, self.gimbal_theta)
        self.gimbal_alpha_history = np.append(self.gimbal_alpha_history, np.linalg.norm(self.gimbal_alpha))
        
    def save_thrust(self, thrust):
        """ Takes in the current Thrust and saves it into the thrust history"""
        self.thrust_history = np.append(self.thrust_history, thrust)
        
    def calculate_angular_rates(self, t):
        if (int(t / self.timestep)) > 1:     
            psi1 = self.gimbal_psi_history[-1]
            theta1 = self.gimbal_theta_history[-1]
            psi2 = self.gimbal_psi_history[-2]
            theta2 = self.gimbal_theta_history[-2]
            psi3 = self.gimbal_psi_history[-3]
            theta3 = self.gimbal_theta_history[-3]
            
            cart1 = np.array([np.sin(psi1)*np.cos(theta1), np.sin(psi1)*np.sin(theta1), -np.cos(psi1)])
            cart2 = np.array([np.sin(psi2)*np.cos(theta2), np.sin(psi2)*np.sin(theta2), -np.cos(psi2)])
            cart3 = np.array([np.sin(psi3)*np.cos(theta3), np.sin(psi3)*np.sin(theta3), -np.cos(psi3)])
            
            w0 = (np.cross(cart1, cart2) / np.linalg.norm(np.cross(cart1, cart2))) * (self.angular_difference(psi1, theta1, psi2, theta2) / self.timestep)
            w1 = (np.cross(cart2, cart3) / np.linalg.norm(np.cross(cart2, cart3))) * (self.angular_difference(psi2, theta2, psi3, theta3) / self.timestep)
            
            self.gimbal_alpha = (w0 - w1) / self.timestep
        
    def angular_difference(self, lat1, lon1, lat2, lon2):
        """
        Calculate the angular difference (in radians) between two points
        given in latitude and longitude.
        
        Parameters:
        lat1, lon1: Latitude and longitude of the first point
        lat2, lon2: Latitude and longitude of the second point
        
        Returns:
        Angular difference (in radians)
        """
        # Convert our engine coords to lat long
        lat1 = (np.pi/2) - lat1
        lat2 = (np.pi/2) - lat2
        
        # Haversine formula to calculate angular difference
        delta_lat = lat2 - lat1
        delta_lon = lon2 - lon1

        a = np.sin(delta_lat / 2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(delta_lon / 2)**2
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
        
        return c
            
