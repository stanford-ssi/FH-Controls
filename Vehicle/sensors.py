from Vehicle.rocketConstants import *
import numpy as np

class Accelerometer():
    def __init__(self):
        self.sigma = ACCELEROMETER_SIGMA
        self.mu = ACCELEROMETER_MU
        
    def reading(self, state):
        return [np.random.normal(x + self.mu, self.sigma) for x in reality]
    
class Gyroscope():
    def __init__(self):
        self.sigma = GYROSCOPE_SIGMA
        self.mu = GYROSCOPE_MU
    
    def reading(self, state):
        return [np.random.normal(x + self.mu, self.sigma) for x in reality]
    
class Barometer():
    def __init__(self):
        self.sigma = BAROMETER_SIGMA
        self.mu = BAROMETER_MU
        
    def reading(self, state):
        return [np.random.normal(x + self.mu, self.sigma) for x in reality]
    
class GPS():
    def __init__(self):
        self.sigma = GPS_SIGMA
        self.mu = GPS_MU
        self.H = 
        
    def reading(self, state):
        return [np.random.normal(x + self.mu, self.sigma) for x in reality]