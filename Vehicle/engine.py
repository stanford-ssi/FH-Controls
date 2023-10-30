import numpy as np
import Vehicle.engineConstants
from scipy.interpolate import interp1d

class Engine:
    """ Class Representing the Engine and associated data"""
    def __init__(self, simulation_timestep):
        # For now, Constant thrust curve determined here, will later be a file from prop that will be imported
        dt_thrust_curve = 0.1 # Seconds
        self.timestep = simulation_timestep
        self.burn_duration = Vehicle.engineConstants.ENGINE_BURN_DURATION
        self.thrust_mass_constant = Vehicle.engineConstants.TRUST_2_MASS_CONSTANT
        self.thrust_curve = np.linspace(2500, 2500, num=int(self.burn_duration/dt_thrust_curve)) #Eventually this will be data in constants file
        self.thrust_history = np.empty(shape=(0)) # The thrust curve we will fill up for integration (Just keep it this way it's easier than trying to reshape the origional thrust curve every time)
        self.thrust = self.thrust_curve[0]

        # Initialize empty list to save throttle history in, to be used for calculating mass of rocket at a given time
        self.throttle_history = np.empty(shape=(0))
        self.throttle = 0
        self.thetax_history = np.empty(shape=(0))
        self.thetax = 0
        self.thetay_history = np.empty(shape=(0))
        self.thetay = 0

        # Masses
        self.drymass = Vehicle.engineConstants.ENGINE_DRYMASS
        self.full_mass = Vehicle.engineConstants.ENGINE_FULL_MASS
        self.mass = self.full_mass

    def get_thrust(self, t, throttle):
        """ Takes in a query time and a throttle percentage and outputs the thrust based on the thrust curve
        
        Inputs:
            t = requested time for query (with t=0 being engine startup)
            throttle = requested throttle percentage
            
        Returns:
            Thrust = Thrust given conditions above
            
        """

        # Calculate Thrust at given time
        maxThrust = self.thrust_curve[int(t/self.timestep)]
        self.thrust = maxThrust

        # Apply Throttling
        Thrust = maxThrust * throttle

        return(Thrust)
    
    def save_throttle(self, throttle):
        """ Takes in the current Throttle and saves it into the throttle history"""
        self.throttle_history = np.append(self.throttle_history, throttle)
        self.throttle = throttle

    def save_thetaX(self, theta_x):
        """ Takes in the current Throttle and saves it into the throttle history"""
        self.thetax_history = np.append(self.thetax_history, theta_x)
        self.thetax = theta_x

    def save_thetaY(self, theta_y):
        """ Takes in the current Throttle and saves it into the throttle history"""
        self.thetay_history = np.append(self.thetay_history, theta_y)
        self.thetay = theta_y
        
    def save_thrust(self, thrust):
        """ Takes in the current Thrust and saves it into the thrust history"""
        self.thrust_history = np.append(self.thrust_history, thrust)
        
    def pull_throttle_history(self):
        pass