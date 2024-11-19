# Constraints on Actuators

# Position Constraints
MAX_POS = 0.1 # meters from centerline of rocket
MAX_THROTTLE = 1 # meters from centerline of rocket
MIN_THROTTLE = 0.2 # percent

# Velocity Constraints
MAX_THROTTLE_RATE = 0.5 # throttles per second, ie 5 --> 500 percent per second
MAX_POS_RATE = 5#0.01# m per second - actuator speed in direction with a bit of saftey margin - should be 0.0092

# Linearization
STEP_SIZE = 0.001

# LQR Controller Penalties for regular flight
Q_X = 50
Q_Y = 50
Q_Z = 0.01
Q_VX = 0.2
Q_VY = 0.2
Q_VZ = 0.02
Q_PIT = 0.2
Q_YAW = 0.2
Q_VPIT = 0.02
Q_VYAW = 0.02
R_X = 0.05
R_Y = 0.05
R_T = 0.075

## Kalman Filter
SIGMA_PROCESS_NOISE = 0.01

## MPC Info
TIMESTEP_HORIZON = 25




