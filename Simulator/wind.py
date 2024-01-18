import random
from Simulator.simulationConstants import *

def get_wind(base_wind, current_wind):
    """ Function that gets the wind at the current timestep. It randomizes the gusts up to 5 times the base wind speed.
        It's randomization is based on two variables defined in the simulation constants wind section
        
        Inputs:
        - base_wind 1x3 array
        - current wind 1x3 array
        
        Outputs:
        - new_wind 1x3 array
        """
        
    if (base_wind==current_wind).all():
        if random.random() > WIND_CHANCE_OF_CHANGE:
            multiplier = 5 * random.random()
            return base_wind * multiplier
        else:
            return base_wind
    else:
        if random.random() > WIND_CHANGE_LENGTH:
            return base_wind
        else:
            return current_wind
        
    
    