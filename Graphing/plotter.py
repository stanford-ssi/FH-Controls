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
    ax.set_xlim(np.min(planned_trajectory[:,0]) - 1, np.max(planned_trajectory[:,0]) + 1)
    ax.set_ylim(np.min(planned_trajectory[:,1]) - 1, np.max(planned_trajectory[:,1]) + 1)
    ax.set_zlim(np.min(planned_trajectory[:,2]) - 1, np.max(planned_trajectory[:,2]) + 1)

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

def dynamics(trajectory, ts, tf):
    frame=len(trajectory)
    # reads x, y, z positions
    x_pos = trajectory[:frame, 0]
    y_pos = trajectory[:frame, 1]
    z_pos = trajectory[:frame, 2]

    # reads x, y, z velocity
    x_vel = trajectory[:frame, 3]
    y_vel = trajectory[:frame, 4]
    z_vel = trajectory[:frame, 5]

    t = np.linspace(0, tf, int(tf/ts)+1)

    # differentiates velocity for x, y, z acceleration
    x_acc = np.diff(x_vel) / np.diff(t[0:len(x_vel)])
    y_acc = np.diff(y_vel) / np.diff(t[0:len(y_vel)])
    z_acc = np.diff(z_vel) / np.diff(t[0:len(z_vel)])

    return [x_pos, y_pos, z_pos, x_vel, y_vel, z_vel, x_acc, y_acc, z_acc]

def plot_variable_vs_time(var, ts, tf, name='INSERT NAME HERE'):

    fig, ax = plt.subplots()

    t = np.linspace(0, tf, int(tf/ts)+1)

    ax.plot(t[0:len(var)], var)

    ax.set_title("%s vs Time"%name)
    ax.set_xlabel("Time")
    ax.set_ylabel(name)

    plt.show()

