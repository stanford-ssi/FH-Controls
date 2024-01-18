import numpy as np

# Simulation Variables
# State is all relative to global frame except Z rotation which is rocket frame
# Pitch is defined as angle from upward z axis towards pos x axis, yaw is angle from upward z towards pos y, and roll is ccw looking down on rocket
# Rotation order yaw, pitch, roll
INITIAL_STATE = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]) # Start State [X, Y, Z, VX, VY, VZ, THX, THY, THZ, OMX, OMY, OMZ]
WIND = np.array([-1, 5, 0])
FINAL_TIME = 20
TIMESTEP = 0.1
TARGET_ALTITUDE = 50 #meters

# Physical Constants
GRAVITY = 9.81 # m/s^2
RHO = 1.225 #kg/m^3

RAD2DEG = 57.2958

# Randomized Wind Variables
WIND_CHANCE_OF_CHANGE = 0.2
WIND_CHANGE_LENGTH = 0.9