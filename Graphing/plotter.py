import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

def animate_3DOF_trajectory(trajectory, planned_trajectory):
    """ Animate 3DOF Plot for 3DOF Trajectory """
    
    # Set up Graph
    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot Placeholder
    line, = ax.plot([], [], [], lw=2, color='blue')
    planned_line, = ax.plot([], [], [], lw=2, color='red')
    
    # Data Bounds
    # NOTE: adjust bounds for plot accordingly
    ax.set_xlim(np.min(planned_trajectory[:,0]), np.max(planned_trajectory[:,0]))
    ax.set_ylim(np.min(planned_trajectory[:,1]), np.max(planned_trajectory[:,1]))
    ax.set_zlim(np.min(planned_trajectory[:,2]), np.max(planned_trajectory[:,2]))

    # Set labels for the axes
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    
    # Initialization function
    def init():
        line.set_data([], [])
        line.set_3d_properties([])

        planned_line.set_data([], [])
        planned_line.set_3d_properties([])
        return line, planned_line
        # return planned_line,
    
    # Update function
    def update(frame):
        x = trajectory[:frame, 0]
        y = trajectory[:frame, 1]
        z = trajectory[:frame, 2]

        # for planned trajectory
        x_planned = planned_trajectory[:frame, 0]
        y_planned = planned_trajectory[:frame, 1]
        z_planned = planned_trajectory[:frame, 2]

        planned_line.set_data(x_planned, y_planned)
        planned_line.set_3d_properties(z_planned)
        ###

        line.set_data(x, y)
        line.set_3d_properties(z)
        return line, planned_line
        # return planned_line,

    # Create Animation
    ani = FuncAnimation(fig, update, frames=range(len(planned_trajectory)), init_func=init, blit=True)

    # Show Animation
    plt.show()

def plot_error(errorHistory):
    # Compute norm of each error vector
    error_norms = np.linalg.norm(errorHistory, axis=1)
    # get array of time steps
    t_array = np.array([i for i in range(error_norms.shape[0])])

    # Plot error history
    fig = plt.figure(2)
    plt.plot(t_array, error_norms)
    plt.xlabel("Time")
    plt.ylabel("Error Norm")
    plt.title("Error vs. Time")
    plt.show()

#mass vs ts plot using matplotlib

def plot_mass(mass, ts, tf):

    fig, ax = plt.subplots()

    t = np.linspace(0, tf, int(tf/ts)+1)

    ax.plot(t, mass)

    ax.set_title("Mass vs. Time")
    ax.set_xlabel("Time")
    ax.set_ylabel("Mass")

    plt.show()

def plot_throttle(throttle, ts, tf):

    fig, ax = plt.subplots()

    t = np.linspace(0, tf, int(tf/ts)+1)

    ax.plot(t, throttle)

    ax.set_title("Throttle vs. Time")
    ax.set_xlabel("Time")
    ax.set_ylabel("Throttle")

    plt.show()
