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
        self.thrust_curve = np.linspace(Vehicle.engineConstants.THRUST_CURVE_VALUE, Vehicle.engineConstants.THRUST_CURVE_VALUE, num=int(self.burn_duration/self.dt_thrust_curve)) #Eventually this will be data in constants file
        self.thrust_history = np.empty(shape=(0)) # The thrust curve we will fill up for integration (Just keep it this way it's easier than trying to reshape the origional thrust curve every time)
        self.thrust = self.thrust_curve[0]

        # Initialize empty list to save throttle history in, to be used for calculating mass of rocket at a given time
        self.throttle_history = np.empty(shape=(0))
        self.throttle = 1
        self.posx_history = np.empty(shape=(0))
        self.posx = 0
        self.posy_history = np.empty(shape=(0))
        self.posy = 0

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
    
    def get_throttle(self, t, thrust):
        """ Takes in a query time and a thrust and outputs the throttle based on the thrust curve
        
        Inputs:
            t = requested time for query (with t=0 being engine startup)
            thrust = requested thrust
            
        Returns:
            throttle = throttle given conditions above
            
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
        if maxThrust == 0:
            throttle = 1
        else:
            throttle = thrust / maxThrust

        return(throttle)
    
    def save_throttle(self, throttle):
        """ Takes in the current Throttle and saves it into the throttle history"""
        self.throttle_history = np.append(self.throttle_history, throttle)
        self.throttle = throttle

    def save_posX(self, posx):
        """ Takes in the current Throttle and saves it into the throttle history"""
        self.posx_history = np.append(self.posx_history, posx)
        self.posx = posx

    def save_posY(self, posy):
        """ Takes in the current Throttle and saves it into the throttle history"""
        self.posy_history = np.append(self.posy_history, posy)
        self.posy = posy
        
    def save_thrust(self, thrust):
        """ Takes in the current Thrust and saves it into the thrust history"""
        self.thrust_history = np.append(self.thrust_history, thrust)
        