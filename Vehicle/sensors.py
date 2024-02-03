from Vehicle.rocketConstants import *
from Vehicle.rocket import *
import numpy as np

class Accelerometer():
    def __init__(self):
        self.sigma = ACCELEROMETER_SIGMA
        self.mu = ACCELEROMETER_MU
        self.dt = ACCELEROMETER_DT
        
    def reading(self, state, real_acc):
        acc_value = real_acc + np.random.normal(self.mu, self.sigma)
        return acc_value
    
    def read_velocity(self, state, real_acc):
        acc = self.reading(state, real_acc)
        new_velocity = state[3:6] + (acc * self.dt)
        return new_velocity
    
    def read_position(self, state, real_acc):
        acc = self.reading(state, real_acc)
        new_velocity = state[3:6] + (acc * self.dt)
        new_position = state[0:3] + (new_velocity * self.dt) + (0.5 * acc * self.dt * self.dt)
        return new_position
    
class Gyroscope():
    def __init__(self):
        self.sigma = GYROSCOPE_SIGMA
        self.mu = GYROSCOPE_MU
        self.dt = GYROSCOPE_DT
    
    def reading(self, state, real_acc):
        acc_value = real_acc + np.random.normal(self.mu, self.sigma) #np.array([np.random.normal(x + self.mu, self.sigma) for x in state[0:3]])
        return acc_value
    
    def read_velocity(self, state, real_acc):
        acc = self.reading(state, real_acc)
        new_velocity = state[9:12] + (acc * self.dt)
        return new_velocity
    
    def read_position(self, state, real_acc):
        acc = self.reading(state, real_acc)
        new_velocity = state[9:12] + (acc * self.dt)
        new_position = state[6:9] + (new_velocity * self.dt) + (0.5 * acc * self.dt * self.dt)
        return new_position
    
class Magnetometer():
    def __init__(self):
        self.sigma = MAGNETOMETER_SIGMA
        self.mu = MAGNETOMETER_MU
        self.dt = MAGNETOMETER_DT
    
    def reading(self, state):
        return np.array([np.random.normal(x + self.mu, self.sigma) for x in state[6:9]])
    
class Barometer():
    def __init__(self):
        self.sigma = BAROMETER_SIGMA
        self.mu = BAROMETER_MU
        
    def reading(self, state):
        return [np.random.normal(x + self.mu, self.sigma) for x in state]
    
class GPS():
    def __init__(self):
        self.sigma = GPS_SIGMA
        self.mu = GPS_MU
        
    def reading(self, state):
        return np.array([np.random.normal(x + self.mu, self.sigma) for x in state[0:3]])