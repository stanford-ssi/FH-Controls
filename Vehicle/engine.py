import numpy as np

class Engine:
    """ Class Representing the Engine and associated data"""
    def __init__(self):
        # For now, Constant thrust curve determined here, will later be a file from prop that will be imported
        self.dataTimeStep = 0.1 # Seconds
        self.burnDuration = 20 # Seconds
        self.thrustCurve = np.linspace(2500, 2500, num=self.burnDuration/self.dataTimeStep)

        # Initialize empty list to save throttle history in, to be used for calculating mass of rocket at a given time
        self.throttleHistory = []

    def get_thrust(self, t, throttle):
        """ Takes in a query time and a throttle percentage and outputs the thrust based on the thrust curve
        
        Inputs:
            t = requested time for query (with t=0 being engine startup)
            throttle = requested throttle percentage
            
        Returns:
            Thrust = Thrust given conditions above
            
        """

        # Calculate Thrust at given time
        maxThrust = self.thrustCurve[t/self.dataTimeStep]

        # Apply Throttling
        Thrust = maxThrust * throttle

        return(Thrust)
    
    def save_throttle(self, throttle):
        """ Takes in the current Throttle and saves it into the throttle history"""
        self.throttleHistory.append(throttle)
        
    def pull_throttle_history(self):
        pass