# Constraints on Actuators

# Position Constraints
MAX_POS = 0.1 # meters from centerline of rocket
MAX_THROTTLE = 1 # meters from centerline of rocket
MIN_THROTTLE = 0.1 # percent

# Velocity Constraints
MAX_THROTTLE_RATE = 0.5 # throttles per second, ie 5 --> 500 percent per second
MAX_POS_RATE = 0.01905 # m per second


# LQR Controller Penalties
Q_X = 0.5
Q_Y = 0.5
Q_Z = 0.02
Q_VX = 1
Q_VY = 1
Q_VZ = 1
Q_PIT = 0.01
Q_YAW = 0.01
Q_VPIT = 0.02
Q_VYAW = 0.02

R_X = 0.01
R_Y = 0.01
R_T = 0.25




