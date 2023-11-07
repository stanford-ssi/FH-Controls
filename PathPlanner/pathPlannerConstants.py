import numpy as np

# Constants and Physics
g = 9.81  # m/s^2, acceleration due to gravity
dt = 0.2  # seconds, time step
N = 100  # number of time steps

# Derived constants
max_a_rate_up = 0.1 * g * dt # m/s^2, maximum upward acceleration
max_a_rate_down = -0.1 * g * dt # m/s^2, maximum downward acceleration 
h_max = 55  # m, maximum height
h_min = 0  # m, minimum height
v_max = 10  # m/s, maximum velocity
v_min = -10  # m/s, minimum velocity
a_max = g  # m/s^2, maximum acceleration
a_min = -g  # m/s^2 minimum acceleration
h_target = 50  # m, target height
v_target = 0  # m/s, target velocity
x0 = np.array([0, 0, 0])  # [m, m, m], initial state vector

#model matrices were removed from here. 
#TO-DO: Discussion on whether to re-add?
