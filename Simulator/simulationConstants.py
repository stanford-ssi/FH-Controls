import numpy as np

# Simulation Variables
# State is all relative to global frame except Z rotation which is rocket frame
# Pitch is defined as angle from upward z axis towards pos x axis, yaw is angle from upward z towards pos y, and roll is ccw looking down on rocket
# Rotation order yaw, pitch, roll
INITIAL_STATE = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]) # Start State [X, Y, Z, VX, VY, VZ, THX, THY, THZ, OMX, OMY, OMZ]
WIND = [5, 5, 0.25] # Sigma values for wind
FINAL_TIME = 20
TIMESTEP = 0.1
TARGET_ALTITUDE = 50 #meters
GROUND_OFFSET = 0.1 # shift the trajectory up by how many meters? (Used to make the controller play nice)

# Physical Constants
GRAVITY = 9.81 # m/s^2
RHO = 1.225 #kg/m^3
RAD2DEG = 57.2958

# Randomized Wind Variables
WIND_CHANCE_OF_CHANGE = 0.01
WIND_CHANGE_LENGTH = 0.96

# Disturbance for engine position and throttle
RANDOMIZED_ERROR_POS = 0.0001 # meter
CONSTANT_ERROR_POS = 0.001 # meter
RANDOMIZED_ERROR_THROTTLE = 0.01
CONSTANT_ERROR_THROTTLE = 0.01

# Random Roll Injection
ROLL_SIGMA = 0.5
ROLL_MU = 0

# Landing Constraints
MAX_RADIUS_ERROR = 1 # m
MAX_Z_SPEED = 0.3 # m/s
MAX_XY_SPEED = 0.05 # m/s
MAX_ROTATION = 0.1 # rad
MAX_ROTATION_SPEED = 0.02 # rad/s
