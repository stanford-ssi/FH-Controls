import random
from Simulator.simulationConstants import *

def get_wind(base_wind, current_wind):
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
        
    
    