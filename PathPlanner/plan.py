#ok but hear me out...

import numpy as np
from scipy.optimize import linprog
import matplotlib.pyplot as plt

# Constants and physics
g = 9.81  # m/s^2
dt = 0.2
N = 100
max_a_rate_up = 0.1 * g * dt
max_a_rate_down = -0.1 * g * dt
h_max = 55  # m
h_min = 0  # m
v_max = 10  # m/s
v_min = -10  # m/s
a_max = g
a_min = -g
h_target = 50
v_target = 0
x0 = [0, 0, 0]

# Model
f = np.zeros(3 * N)
f[2::3] = 1
Phi = np.array([[1, dt],
                [0, 1]])
Gamma = np.array([0, dt])

# Optimizer setup
A = np.zeros((6 * N, 3 * N))
B = np.zeros(6 * N)
for i in range(N):
    A[3 * i, 3 * i] = 1
    A[3 * i + 1, 3 * i + 1] = 1
    A[3 * i + 2, 3 * i + 2] = 1
    B[3 * i] = h_max
    B[3 * i + 1] = v_max
    B[3 * i + 2] = a_max
    A[3 * N + 3 * i, 3 * i] = -1
    A[3 * N + 3 * i + 1, 3 * i + 1] = -1
    A[3 * N + 3 * i + 2, 3 * i + 2] = -1
    B[3 * N + 3 * i] = -h_min
    B[3 * N + 3 * i + 1] = -v_min
    B[3 * N + 3 * i + 2] = -a_min

for i in range(N - 1):
    A[6 * N + 2 * i, 3 * i + 2] = 1
    A[6 * N + 2 * i, 3 * (i + 1) + 2] = -1
    B[6 * N + 2 * i] = -max_a_rate_down
    A[6 * N + 2 * i + 1, 3 * i + 2] = -1
    A[6 * N + 2 * i + 1, 3 * (i + 1) + 2] = 1
    B[6 * N + 2 * i + 1] = max_a_rate_up

# Equality constraints
A_eq = np.zeros((2 * N + 4, 3 * N))
B_eq = np.zeros(2 * N + 4)
physics_mat = np.zeros((3, 3))
physics_mat[0:2, 0:2] = Phi
physics_mat[0:2, 2] = Gamma

for i in range(N - 1):
    A_eq[2 * i:2 * i + 2, 3 * i:3 * i + 3] = -physics_mat[0:2, :]
    A_eq[2 * i:2 * i + 2, 3 * (i + 1):3 * (i + 1) + 2] = np.eye(2)

A_eq[2 * N - 3:2 * N - 1, 3 * N] = 0
A_eq[2 * N - 1, 0] = 1
A_eq[2 * N, 1] = 1
A_eq[2 * N + 1, 2] = 1
B_eq[2 * N - 1:2 * N + 1] = x0
A_eq[2 * N + 2, 3 * N - 3] = 1
A_eq[2 * N + 3, 3 * N - 2] = 1
B_eq[2 * N + 2] = h_target
B_eq[2 * N + 3] = v_target

# Solving linear programming problem
result = linprog(f, A_ub=A, b_ub=B, A_eq=A_eq, b_eq=B_eq, method='highs')

x = result.x

# Plotting
t = dt * np.linspace(0, N - 1, N)
plt.figure(1)
plt.plot(t, x[0::3])
plt.xlabel('Time (s)')
plt.ylabel('Height (m)')

plt.figure(2)
plt.plot(t, x[1::3])
plt.xlabel('Time (s)')
plt.ylabel('Vertical Speed (m/s)')

plt.figure(3)
plt.plot(t, x[2::3])
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s^2)')

plt.show()

