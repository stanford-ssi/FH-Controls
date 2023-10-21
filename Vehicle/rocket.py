import numpy as np
import Vehicle.engine

class Rocket:
    """ Class Representing the Rocket and associated data"""
    def __init__(self):
        # Create Engine Object inside Rocket
        self.engine = Vehicle.engine.Engine()
        
    def get_mass(self, throttleHistory):
        """ Takes in a throttle history and outputs the expected mass based on it
        
        Inputs:
            throttleHistory = history of throttle
            
        Returns:
            mass = mass of rocket at current query time
            
        """
        
        return(50)