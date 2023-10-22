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
        
        # Match Thrust Curve Indicies to simulation steprate
        interpolation_function = interp1d(np.arange(len(self.thrustCurve)), self.thrustCurve, kind='linear')
        new_indices = np.linspace(0, len(self.thrustCurve) - 1, int(self.burnDuration/self.timestep))  # Change 10 to the desired number of indices
        self.thrustCurve = interpolation_function(new_indices)

        # Initialize empty list to save throttle history in, to be used for calculating mass of rocket at a given time
        self.throttleHistory = np.array((0,0))

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
        np.append(self.throttleHistory, throttle)
        
    def pull_throttle_history(self):
        pass