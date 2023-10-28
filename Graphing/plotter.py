import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

def animate_3DOF_trajectory(trajectory):
    """ Animate 3DOF Plot for 3DOF Trajectory """
    
    # Set up Graph
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot Placeholder
    line, = ax.plot([], [], [], lw=2)
    
    # Data Bounds
    ax.set_xlim(np.min(trajectory[:,0]), np.max(trajectory[:,0]))
    ax.set_ylim(np.min(trajectory[:,1]), np.max(trajectory[:,1]))
    ax.set_zlim(np.min(trajectory[:,2]), np.max(trajectory[:,2]))

    # Set labels for the axes
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    
    # Initialization function
    def init():
        line.set_data([], [])
        line.set_3d_properties([])
        return line,
    
    # Update function
    def update(frame):
        x = trajectory[:frame, 0]
        y = trajectory[:frame, 1]
        z = trajectory[:frame, 2]
        line.set_data(x, y)
        line.set_3d_properties(z)
        return line,

    # Create Animation
    ani = FuncAnimation(fig, update, frames=range(len(trajectory)), init_func=init, blit=True)

    # Show Animation
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