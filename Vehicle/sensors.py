from Vehicle.rocketConstants import *
from Vehicle.rocket import *
import numpy as np
import copy

class Accelerometer():
    """ Accelerometer Object """
    def __init__(self, state):
        self.sigma = ACCELEROMETER_SIGMA
        self.mu = ACCELEROMETER_MU
        self.dt = ACCELEROMETER_UPDATE_FREQ
        self.update_time = 1 / ACCELEROMETER_UPDATE_FREQ
        
        # Readings Setup
        self.last_measurement = np.array([0,0,0])
        self.last_measurement = self.read_velocity(state, [0,0,0])
        self.last_measurement_t = 0
        
    def get_acc(self, real_acc):
        """ Simulates a reading from the accelerometer
        
        Inputs:
        - real acceleration vector (1x3)
        
        Outputs:
        - acceleration vector with x y and z (1x3)  
        
        """
        return np.array([np.random.normal(x + self.mu, self.sigma) for x in real_acc])
    
    def read_velocity(self, state, real_acc):
        """ Gets a velocity reading from the accelerometer
        
        Inputs:
        - state vector (1x12)
        - real acceleration vector (1x3)
        
        Outputs:
        - velocity vector with p y and r (1x3)  
        
        """
        acc = self.get_acc(real_acc)
        new_velocity = state[3:6] + (acc * self.update_time)
        return new_velocity
    
    def read_position(self, state, real_acc):
        """ Gets a position reading from the accelerometer
        
        Inputs:
        - state vector (1x12)
        - real acceleration vector (1x3)
        
        Outputs:
        - position vector with p y and r (1x3)  
        
        """
        acc = self.get_acc(real_acc)
        new_velocity = state[3:6] + (acc * self.dt)
        new_position = state[0:3] + (new_velocity * self.dt) + (0.5 * acc * self.dt * self.dt)
        return new_position
    
    def reading(self, state, acc, t):
        """ Simulates a reading from the GPS
        
        Inputs:
        - state vector (1x12)
        
        Outputs:
        - vector with x y and z, xdot, ydot, zdot (1x6)  
        
        """
        if t > self.last_measurement_t + self.update_time or t == 0:
            self.last_measurement_t = t
            self.last_measurement = self.read_velocity(state, acc)
            return self.last_measurement
        else:
            return np.array([np.nan, np.nan, np.nan])
    
class Gyroscope():
    """ Gyroscope Object """
    def __init__(self, state):
        # Parameter Setup
        self.sigma = GYROSCOPE_SIGMA
        self.mu = GYROSCOPE_MU
        self.update_time = 1 / GYROSCOPE_UPDATE_FREQ
        
        # Readings Setup
        self.last_measurement = np.array([0,0,0])
        self.last_measurement = self.update_reading(state)
        self.last_measurement_t = 0
        
    def update_reading(self, state):
        return np.array([np.random.normal(x + self.mu, self.sigma) for x in state[9:12]])
        
    def reading(self, state, t):
        """ Simulates a reading from the GPS
        
        Inputs:
        - state vector (1x12)
        
        Outputs:
        - vector with x y and z, xdot, ydot, zdot (1x6)  
        
        """
        if t > self.last_measurement_t + self.update_time or t == 0:
            self.last_measurement_t = t
            self.last_measurement = self.update_reading(state)
            return self.last_measurement
        else:
            return np.array([np.nan, np.nan, np.nan])
    
class Magnetometer():
    """ Magnetometer Object """
    def __init__(self, state):
        # Parameter Setup
        self.sigma = MAGNETOMETER_SIGMA
        self.mu = MAGNETOMETER_MU
        self.update_time = 1 / MAGNETOMETER_UPDATE_FREQ
        
        # Readings Setup
        self.last_measurement = np.array([0,0,0])
        self.last_measurement = self.update_reading(state)
        self.last_measurement_t = 0
        
    def update_reading(self, state):
        return np.array([np.random.normal(x + self.mu, self.sigma) for x in state[6:9]])
        
    def reading(self, state, t):
        """ Simulates a reading from the GPS
        
        Inputs:
        - state vector (1x12)
        
        Outputs:
        - vector with x y and z, xdot, ydot, zdot (1x6)  
        
        """
        if t > self.last_measurement_t + self.update_time or t == 0:
            self.last_measurement_t = t
            self.last_measurement = self.update_reading(state)
            return self.last_measurement
        else:
            return np.array([np.nan, np.nan, np.nan])
    
class Barometer():
    """ Barometer Object """
    def __init__(self, state):
        # Parameter Setup
        self.sigma = BAROMETER_SIGMA
        self.mu = BAROMETER_MU
        self.update_time = 1 / BAROMETER_UPDATE_FREQ
        
        # Readings Setup
        self.last_measurement = np.array([0])
        self.last_measurement = self.update_reading(state)
        self.last_measurement_t = 0
        
    def update_reading(self, state):
        return np.array([np.random.normal(state[2] + self.mu, self.sigma)])
        
    def reading(self, state, t):
        """ Simulates a reading from the GPS
        
        Inputs:
        - state vector (1x12)
        
        Outputs:
        - vector with x y and z, xdot, ydot, zdot (1x6)  
        
        """
        if t > self.last_measurement_t + self.update_time or t == 0:
            self.last_measurement_t = t
            self.last_measurement = self.update_reading(state)
            return self.last_measurement
        else:
            return np.array([np.nan])
    
class GPS():
    """ GPS Object """
    def __init__(self, state):
        # Parameter Setup
        self.sigma = GPS_SIGMA
        self.mu = GPS_MU
        self.update_time = 1 / GPS_UPDATE_FREQ
        
        # Readings Setup
        self.last_measurement = np.array([0,0,0,0,0,0])
        self.last_measurement = self.update_reading(state[0:3])
        self.last_measurement_t = 0
        
    def update_reading(self, state):
        pos = np.array([np.random.normal(x + self.mu, self.sigma) for x in state[0:3]])
        vel = np.array([np.random.normal(x + self.mu, self.sigma) for x in state[3:6]])
        return np.hstack((pos, vel))
        
    def reading(self, state, t):
        """ Simulates a reading from the GPS
        
        Inputs:
        - state vector (1x12)
        
        Outputs:
        - vector with x y and z, xdot, ydot, zdot (1x6)  
        
        """
        if t > self.last_measurement_t + self.update_time or t == 0:
            self.last_measurement_t = t
            self.last_measurement = self.update_reading(state)
            return self.last_measurement
        else:
            return np.array([np.nan, np.nan, np.nan, np.nan, np.nan, np.nan])