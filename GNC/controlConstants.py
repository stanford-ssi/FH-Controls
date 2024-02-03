# Constraints on Actuators

# Position Constraints
MAX_POS = 0.1 # meters from centerline of rocket
MAX_THROTTLE = 1 # meters from centerline of rocket
MIN_THROTTLE = 0.2 # percent

# Velocity Constraints
MAX_THROTTLE_RATE = 0.5 # throttles per second, ie 5 --> 500 percent per second
MAX_POS_RATE = 0.01# m per second - actuator speed in direction with a bit of saftey margin - should be 0.0092


# LQR Controller Penalties for regular flight
Q_X = 0.5
Q_Y = 0.5
Q_Z = 0.1
Q_VX = 2
Q_VY = 2
Q_VZ = 0.1
Q_PIT = 0.01
Q_YAW = 0.01
Q_VPIT = 0.01
Q_VYAW = 0.01

R_X = 0.05
R_Y = 0.05
R_T = 0.075

# LQR Controller Penalties for landing phase
Q_X_l = 0.1
Q_Y_l = 0.1
Q_Z_l = 0.005
Q_VX_l = 1
Q_VY_l = 1
Q_VZ_l = 0.05
Q_PIT_l = 0.01
Q_YAW_l = 0.01
Q_VPIT_l = 0.01
Q_VYAW_l = 0.01

R_X_l = 0.05
R_Y_l = 0.05
R_T_l = 0.075


## Kalman Filter
SIGMA_PROCESS_NOISE = 0.01




