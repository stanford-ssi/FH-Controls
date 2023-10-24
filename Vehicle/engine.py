import numpy as np
import Vehicle.engineConstants
from scipy.interpolate import interp1d

class Engine:
    """ Class Representing the Engine and associated data"""
    def __init__(self, simulation_timestep):
        # For now, Constant thrust curve determined here, will later be a file from prop that will be imported
        dtThrustCurve = 0.1 # Seconds
        self.timestep = simulation_timestep
        self.burnDuration = Vehicle.engineConstants.ENGINE_BURN_DURATION
        self.thrustCurve = np.linspace(2500, 2500, num=int(self.burnDuration/dtThrustCurve)) #Eventually this will be data in constants file
        self.thrustHistory = np.empty(shape=(0)) # The thrust curve we will fill up for integration (Just keep it this way it's easier than trying to reshape the origional thrust curve every time)
        
        # Initialize empty list to save throttle history in, to be used for calculating mass of rocket at a given time
        self.throttleHistory = np.empty(shape=(0))
        self.throttle = 1

        # Masses
        self.drymass = Vehicle.engineConstants.ENGINE_DRYMASS
        self.fullMass = Vehicle.engineConstants.ENGINE_FULL_MASS
        self.mass = self.fullMass

    def get_thrust(self, t, throttle):
        """ Takes in a query time and a throttle percentage and outputs the thrust based on the thrust curve
        
        Inputs:
            t = requested time for query (with t=0 being engine startup)
            throttle = requested throttle percentage
            
        Returns:
            Thrust = Thrust given conditions above
            
        """

        # Calculate Thrust at given time
        maxThrust = self.thrustCurve[int(t/self.timestep)]

        # Apply Throttling
        Thrust = maxThrust * throttle

        return(Thrust)
    
    def save_throttle(self, throttle):
        """ Takes in the current Throttle and saves it into the throttle history"""
        self.throttleHistory = np.append(self.throttleHistory, throttle)
        self.throttle = throttle
        
    def save_thrust(self, thrust):
        """ Takes in the current Thrust and saves it into the thrust history"""
        self.thrustHistory = np.append(self.thrustHistory, thrust)
        
    def pull_throttle_history(self):
        pass