import numpy as np

# Simulation Variables
# State is all relative to global frame except Z rotation which is rocket frame
# Pitch is defined as angle from upward z axis towards pos x axis, yaw is angle from upward z towards pos y, and roll is ccw looking down on rocket
# Rotation order yaw, pitch, roll
INITIAL_STATE = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]) # Start State [X, Y, Z, VX, VY, VZ, THX, THY, THZ, WX, WY, WZ]
WIND = [5, 5, 0.25] # Sigma values for wind
FINAL_TIME = 20
TIMESTEP = 0.1
TARGET_ALTITUDE = 50 #meters
GROUND_OFFSET = 0.05 # shift the trajectory up by how many meters? (Used to make the controller play nice)

# Physical Constants
GRAVITY = 9.81 # m/s^2
RHO = 1.225 #kg/m^3
RAD2DEG = 57.2958

# Randomized Wind Variables
WIND_CHANCE_OF_CHANGE = 0.01
WIND_CHANGE_LENGTH = 0.96
WIND_MULTIPLIER = 1.5

# Disturbance for engine position and throttle
RANDOMIZED_ERROR_POS_SIGMA = 0.0001 # meter
RANDOMIZED_ERROR_THROTTLE_SIGMA = 0.01
CONSTANT_ERROR_THROTTLE = 0.0

# Random Roll Injection
ROLL_SIGMA = 0.1
ROLL_MU = 0

# Landing Constraints
THRESHOLD_ALTITUDE = 1
THRESHOLD_TIME = 0.95 * FINAL_TIME
MAX_RADIUS_ERROR = 3 # m
MAX_Z_SPEED = 2 # m/s
MAX_XY_SPEED = 0.2 # m/s
MAX_ROTATION = 0.1 # rad
MAX_ROTATION_SPEED = 0.1 # rad/s
