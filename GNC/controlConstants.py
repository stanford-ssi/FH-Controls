# Constraints on Actuators

# Position Constraints
MAX_POS = 0.1 # meters from centerline of rocket
MAX_THROTTLE = 1 # meters from centerline of rocket
MIN_THROTTLE = 0.1 # percent

# Velocity Constraints
MAX_THROTTLE_RATE = 0.5 # throttles per second, ie 5 --> 500 percent per second
MAX_POS_RATE = 0.01905 # m per second


# LQR Controller Penalties
Q_Z_POS = 1000 # Penalty in z error
Q_Z_VEL = 100  # Penalty in z velocity error
R_THROTTLE = 5 # Penalty for throttle control
R_GIMBALS = 5 # Penalty for gimbal control




