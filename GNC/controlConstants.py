# Constraints on Actuators

# Position Constraints
MAX_POS = 0.1 # meters from centerline of rocket
MAX_THROTTLE = 1 # meters from centerline of rocket
MIN_THROTTLE = 0.2 # percent

# Velocity Constraints
MAX_THROTTLE_RATE = 0.5 # throttles per second, ie 5 --> 500 percent per second
MAX_POS_RATE = 0.0085# m per second - actuator speed in direction with a bit of saftey margin - should be 0.0092


# LQR Controller Penalties
Q_X = 0.1
Q_Y = 0.1
Q_Z = 0.005
Q_VX = 1
Q_VY = 1
Q_VZ = 0.5
Q_PIT = 0.01
Q_YAW = 0.01
Q_VPIT = 0.01
Q_VYAW = 0.01

R_X = 0.002
R_Y = 0.002
R_T = 0.075

## Kalman Filter
SIGMA_PROCESS_NOISE = 0.2




