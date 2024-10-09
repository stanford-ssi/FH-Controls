
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider
import random


def plotVectorsOverTime(bodyAxesX, bodyAxesY, bodyAxesZ, engine_thrust, time):

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    quiver_body_axesX = ax.quiver(0, 0, 0, bodyAxesX[0, 0], bodyAxesX[1, 0], bodyAxesX[2, 0], color='c', label='Body Axes X')
    quiver_body_axesY = ax.quiver(0, 0, 0, bodyAxesY[0, 0], bodyAxesY[1, 0], bodyAxesY[2, 0], color='m', label='Body Axes Y')
    quiver_body_axesZ = ax.quiver(0, 0, 0, bodyAxesZ[0, 0], bodyAxesZ[1, 0], bodyAxesZ[2, 0], color='y', label='Body Axes Z')
    quiver_engine_thrust = ax.quiver(0, 0, 0, engine_thrust[0, 0], engine_thrust[1, 0], engine_thrust[2, 0], color='r', label='Thrust')

    ax.set_xlim([-1, 1])  
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Vectors in 3D Space at Time: {:.2f}'.format(time[0]))
    ax.legend()

    ax_time = plt.axes([0.1, 0.01, 0.8, 0.03], facecolor='lightgoldenrodyellow')

    slider = Slider(ax_time, 'Time', 0, len(time) - 1, valinit=0, valstep=1)

    def update(val):
        index = int(slider.val)

        quiver_body_axesX.set_segments([np.array([[0, 0, 0], bodyAxesX[index]])])
        quiver_body_axesY.set_segments([np.array([[0, 0, 0], bodyAxesY[index]])])
        quiver_body_axesZ.set_segments([np.array([[0, 0, 0], bodyAxesZ[index]])])
        quiver_engine_thrust.set_segments([np.array([[0, 0, 0], engine_thrust[index]])])

        ax.set_title('Vectors in 3D Space at Time: {:.2f}'.format(time[index]))
        fig.canvas.draw_idle()

    slider.on_changed(update)
    plt.show()

def extract_columns(matrix):
    column1 = matrix[:,0,:]
    column2 = matrix[:,1,:]
    column3 = matrix[:,2,:]

    column1 = np.expand_dims(column1, axis=0)
    column2 = np.expand_dims(column2, axis=0)
    column3 = np.expand_dims(column3, axis=0)

    return column1[0, 1:len(column1[0])-1], column2[0, 1:len(column2[0])-1], column3[0, 1:len(column3[0])-1]
    

def plot_frames_over_time(sim):
    time = np.linspace(0, round(sim.previous_time, 1), len(sim.rocket.error_history) - 1)
    bodyAxesX, bodyAxesY, bodyAxesZ = extract_columns(np.linalg.inv(sim.rocket.R_history))

    x = sim.rocket.engine.posx_history
    y = sim.rocket.engine.posy_history
    z = -np.sqrt(sim.rocket.engine.throttle_history ** 2 - np.sqrt(x**2 + y**2))

    engine_thrust = (np.vstack((x, y, z)).T @ [np.linalg.inv(matrix) for matrix in sim.rocket.R_history])[0]
    

    plotVectorsOverTime(bodyAxesX, bodyAxesY, bodyAxesZ, engine_thrust, time)