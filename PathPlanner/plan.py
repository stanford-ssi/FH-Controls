import numpy as np
from scipy.optimize import linprog
import matplotlib.pyplot as plt
from pathPlannerConstants import * 

class PathPlanner:
    def __init__(self, N, dt, x0, h_target, v_target, h_max, h_min, v_max, v_min, a_max, a_min, max_a_rate_up, max_a_rate_down):
        # Initialize all required variables
        self.N = N
        self.dt = dt
        self.x0 = x0
        self.h_target = h_target
        self.v_target = v_target
        self.h_max = h_max
        self.h_min = h_min
        self.v_max = v_max
        self.v_min = v_min
        self.a_max = a_max
        self.a_min = a_min
        self.max_a_rate_up = max_a_rate_up
        self.max_a_rate_down = max_a_rate_down
        self.f = np.zeros(3 * N)
        self.A = np.zeros((6 * N, 3 * N))
        self.B = np.zeros(6 * N)
        self.A_eq = np.zeros((2 * N + 4, 3 * N))
        self.B_eq = np.zeros(2 * N + 4)
        self.result = None
        self.setup_problem()

    def setup_problem(self):
        # Setup cost function, inequality and equality constraints
        self.construct_cost_function()
        self.setup_inequality_constraints()
        self.setup_equality_constraints()

    def construct_cost_function(self):
        # Construct the cost function vector f
        self.f[2::3] = 1  # Only considering the acceleration components for cost

    def setup_inequality_constraints(self):
        # Initialize and set up inequality constraints A and B
        for i in range(self.N):
            idx = 3 * i
            self.A[idx:idx+3, idx:idx+3] = np.eye(3)
            self.B[idx:idx+3] = [self.h_max, self.v_max, self.a_max]
            self.A[idx+3*self.N:idx+3+3*self.N, idx:idx+3] = -np.eye(3)
            self.B[idx+3*self.N:idx+3+3*self.N] = [-self.h_min, -self.v_min, -self.a_min]

        # Add rate of change constraints
        for i in range(1, self.N):
            idx = 2 * (i - 1) + 6 * self.N
            self.A[idx, 3 * i] = 1
            self.A[idx, 3 * (i + 1)] = -1
            self.B[idx] = -self.max_a_rate_down
            self.A[idx + 1, 3 * i] = -1
            self.A[idx + 1, 3 * (i + 1)] = 1
            self.B[idx + 1] = self.max_a_rate_up

    def setup_equality_constraints(self):
        # Initialize and set up equality constraints A_eq and B_eq
        physics_mat = np.zeros((3, 3))
        physics_mat[:2, :2] = Phi
        physics_mat[:2, 2] = Gamma

        for i in range(self.N - 1):
            idx = 2 * i
            self.A_eq[idx:idx+2, 3*i:3*i+3] = -physics_mat[:2, :3]
            self.A_eq[idx:idx+2, 3*(i+1):3*(i+1)+2] = np.eye(2)

        self.A_eq[2 * self.N - 1:2 * self.N + 1, :3] = np.eye(3)
        self.B_eq[2 * self.N - 1:2 * self.N + 1] = self.x0

        self.A_eq[2 * self.N + 2, 3*self.N-3] = 1
        self.A_eq[2 * self.N + 3, 3*self.N-2] = 1
        self.B_eq[2 * self.N + 2] = self.h_target
        self.B_eq[2 * self.N + 3] = self.v_target

    def solve_optimization_problem(self):
        # Solve the optimization problem using linprog
        # Since scipy.optimize.linprog minimizes, we negate f to maximize
        self.result = linprog(-self.f, A_ub=self.A, b_ub=self.B, A_eq=self.A_eq, b_eq=self.B_eq, method='highs')

    def plot_results(self):
        # Extract the solution and plot the results
        x = self.result.x

        time = np.linspace(0, self.N*self.dt, self.N)
        plt.figure(1)
        plt.plot(time, x[::3])
        plt.xlabel('Time (s)')
        plt.ylabel('Height (m)')

        plt.figure(2)
        plt.plot(time, x[1::3])
        plt.xlabel('Time (s)')
        plt.ylabel('Vertical Speed (m/s)')

        plt.figure(3)
        plt.plot(time, x[2::3])
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (m/s^2)')

        plt.show()

# Assuming pathPlannerConstants.py provides the constants like N, dt, x0, h_max, etc.
planner = PathPlanner(N, dt, x0, h_target, v_target, h_max, h_min, v_max, v_min, a_max, a_min, max_a_rate_up, max_a_rate_down)
planner.solve_optimization_problem()
planner.plot_results()
