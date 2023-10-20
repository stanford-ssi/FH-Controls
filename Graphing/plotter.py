import matplotlib.pyplot as plt

def plot_3DOF_trajectory(trajectory):
    """ Simple 3DOF Plot for 3DOF Trajectory """
    
    # Set up Graph
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Pull Data
    x = trajectory[:,0]
    y = trajectory[:,1]
    z = trajectory[:,2]

    # Plot the Trajectory
    ax.plot(x, y, z)

    # Set labels for the axes
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')

    # Show the Plot
    plt.show()
