from Vehicle.rocketConstants import *
from Vehicle.rocket import *
import numpy as np

class Accelerometer():
    """ Accelerometer Object """
    def __init__(self):
        self.sigma = ACCELEROMETER_SIGMA
        self.mu = ACCELEROMETER_MU
        self.dt = ACCELEROMETER_DT
        
    def reading(self, real_acc):
        """ Simulates a reading from the accelerometer
        
        Inputs:
        - real acceleration vector (1x3)
        
        Outputs:
        - acceleration vector with x y and z (1x3)  
        
        """
        acc_value = real_acc + np.random.normal(self.mu, self.sigma)
        return acc_value
    
    def read_velocity(self, state, real_acc):
        """ Gets a velocity reading from the accelerometer
        
        Inputs:
        - state vector (1x12)
        - real acceleration vector (1x3)
        
        Outputs:
        - velocity vector with p y and r (1x3)  
        
        """
        acc = self.reading(real_acc)
        new_velocity = state[3:6] + (acc * self.dt)
        return new_velocity
    
    def read_position(self, state, real_acc):
        """ Gets a position reading from the accelerometer
        
        Inputs:
        - state vector (1x12)
        - real acceleration vector (1x3)
        
        Outputs:
        - position vector with p y and r (1x3)  
        
        """
        acc = self.reading(real_acc)
        new_velocity = state[3:6] + (acc * self.dt)
        new_position = state[0:3] + (new_velocity * self.dt) + (0.5 * acc * self.dt * self.dt)
        return new_position
    
class Gyroscope():
    """ Gyroscope Object """
    def __init__(self):
        self.sigma = GYROSCOPE_SIGMA
        self.mu = GYROSCOPE_MU
        self.dt = GYROSCOPE_UPDATE_FREQ
    
    def reading(self, real_acc):
        """ Simulates a reading from the gyroscope
        
        Inputs:
        - real acceleration vector (1x3)
        
        Outputs:
        - rotational accelerometer vector with p y and r (1x3)  
        
        """
        acc_value = real_acc + np.random.normal(self.mu, self.sigma) #np.array([np.random.normal(x + self.mu, self.sigma) for x in state[0:3]])
        return acc_value
    
    def read_velocity(self, state, real_acc):
        """ Gets a rotational velocity reading from the gyro
        
        Inputs:
        - state vector (1x12)
        - real acceleration vector (1x3)
        
        Outputs:
        - rotational velocity vector with p y and r (1x3)  
        
        """
        acc = self.reading(real_acc)
        new_velocity = state[9:12] + (acc * self.dt)
        return new_velocity
    
    def read_position(self, state, real_acc):
        """ Gets a rotational position reading from the gyro
        
        Inputs:
        - state vector (1x12)
        - real acceleration vector (1x3)
        
        Outputs:
        - rotational position vector with p y and r (1x3)  
        
        """
        acc = self.reading(real_acc)
        new_velocity = state[9:12] + (acc * self.dt)
        new_position = state[6:9] + (new_velocity * self.dt) + (0.5 * acc * self.dt * self.dt)
        return new_position
    
class Magnetometer():
    """ Magnetometer Object """
    def __init__(self):
        self.sigma = MAGNETOMETER_SIGMA
        self.mu = MAGNETOMETER_MU
        self.dt = MAGNETOMETER_DT
    
    def reading(self, state):
        """ Simulates a reading from the magnetometer
        
        Inputs:
        - state vector (1x12)
        
        Outputs:
        - rotational position vector with p y and r (1x3)  
        
        """
        return np.array([np.random.normal(x + self.mu, self.sigma) for x in state[6:9]])
    
class Barometer():
    """ Barometer Object """
    def __init__(self):
        self.sigma = BAROMETER_SIGMA
        self.mu = BAROMETER_MU
        
    def reading(self, state):
        """ Simulates a reading from the Barometer
        
        Inputs:
        - state vector (1x12)
        
        Outputs:
        - z altitutde 
        
        """
        return np.random.normal(state[2] + self.mu, self.sigma)
    
class GPS():
    """ GPS Object """
    def __init__(self, state):
        self.sigma = GPS_SIGMA
        self.mu = GPS_MU
        self.update_time = 1 / GPS_UPDATE_FREQ
        self.data = self.update_reading(state[0:3])
        self.last_measurement_t = 0
        
    def update_reading(self, state):
        return np.array([np.random.normal(x + self.mu, self.sigma) for x in state[0:3]])
        
    def reading(self, state, t):
        """ Simulates a reading from the GPS
        
        Inputs:
        - state vector (1x12)
        
        Outputs:
        - position vector with x y and z (1x3)  
        
        """
        if t > self.last_measurement_t + self.update_time:
            self.last_measurement_t = t
            self.data = self.update_reading(state)
            return self.data
        else:
            return self.data