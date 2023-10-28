import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

def animate_3DOF_trajectory(trajectory, planned_trajectory):
    """ Animate 3DOF Plot for 3DOF Trajectory """
    
    # Set up Graph
    fig = plt.figure()
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
