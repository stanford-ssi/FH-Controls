from rocketConstants import *
import numpy as np

class Accelerometer():
    def __init__(self):
        self.sigma = ACCELEROMETER_SIGMA
        
    def reading(self, reality):
        return [np.random.normal(x, self.sigma) for x in reality]
    
class Gyroscope():
    def __init__(self):
        self.sigma = GYROSCOPE_SIGMA
    
    def reading(self, reality):
        return [np.random.normal(x, self.sigma) for x in reality]
    
class Barometer():
    def __init__(self):
        self.sigma = BAROMETER_SIGMA
    
    def reading(self, reality):
        return [np.random.normal(x, self.sigma) for x in reality]