# Constraints on Actuators

# Position Constraints
MAX_POS = 0.1 # meters from centerline of rocket
MAX_THROTTLE = 1 # meters from centerline of rocket
MIN_THROTTLE = 0.1 # percent

# Velocity Constraints
MAX_THROTTLE_RATE = 5 # throttles per second, ie 5 --> 500 percent per second
MAX_POS_RATE = 2 # m per second


# LQR Controller Penalties
Q_Z_POS = 1000 # Weight in z position
Q_Z_VEL = 100  # Weight in z velocity
R_THROTTLE = 0.75 # Weight for throttle control




