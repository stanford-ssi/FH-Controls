import numpy as np
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from Simulator.simulationConstants import *
from Vehicle.rocketConstants import *
import math
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider

def pull_dynamics(trajectory, ts, tf):
    frame=len(trajectory)
    # reads x, y, z positions
    x_pos = trajectory[:frame, 0]
    y_pos = trajectory[:frame, 1]
    z_pos = trajectory[:frame, 2]
    theta_x = trajectory[:frame, 6]
    theta_y = trajectory[:frame, 7]
    theta_z = trajectory[:frame, 8]

    # reads x, y, z velocity
    x_vel = trajectory[:frame, 3]
    y_vel = trajectory[:frame, 4]
    z_vel = trajectory[:frame, 5]
    omega_x = trajectory[:frame, 9]
    omega_y = trajectory[:frame, 10]
    omega_z = trajectory[:frame, 11]

    t = np.linspace(0, tf, int(tf/ts)+1)

    # differentiates velocity for x, y, z acceleration
    try:
        x_acc = np.diff(x_vel) / np.diff(t[0:len(x_vel)])
        y_acc = np.diff(y_vel) / np.diff(t[0:len(y_vel)])
        z_acc = np.diff(z_vel) / np.diff(t[0:len(z_vel)])
        x_alpha = np.diff(omega_x) / np.diff(t[0:len(omega_x)])
        y_alpha = np.diff(omega_y) / np.diff(t[0:len(omega_y)])
        z_alpha = np.diff(omega_z) / np.diff(t[0:len(omega_z)])
    except:
        x_acc = np.diff(x_vel[0:-1]) / np.diff(t[0:len(x_vel)])
        y_acc = np.diff(y_vel[0:-1]) / np.diff(t[0:len(y_vel)])
        z_acc = np.diff(z_vel[0:-1]) / np.diff(t[0:len(z_vel)])
        x_alpha = np.diff(omega_x[0:-1]) / np.diff(t[0:len(omega_x)])
        y_alpha = np.diff(omega_y[0:-1]) / np.diff(t[0:len(omega_y)])
        z_alpha = np.diff(omega_z[0:-1]) / np.diff(t[0:len(omega_z)])
        

    return [x_pos, y_pos, z_pos, x_vel, y_vel, z_vel, x_acc, y_acc, z_acc, theta_x, theta_y, theta_z, omega_x, omega_y, omega_z, x_alpha, y_alpha, z_alpha]

def pull_sensed_dynamics(sensed_state_history, ts, tf):
    positional = {"GPS": sensed_state_history[:,0:6],
                  "Barometer": sensed_state_history[:,6],
                  "IMU": sensed_state_history[:,7:10]}
    rotational = {"Magnetometer": sensed_state_history[:,10:13] * RAD2DEG,
                  "Gyro": sensed_state_history[:,13:16] * RAD2DEG}
    return positional, rotational

def create_graph_set(tab, var, ts, tf, names, num_graphs, legend, multiple_on_one_graph=False, sensors=None):
    #this figures out how to chunk the graphs to make them easily visible
    if math.sqrt(num_graphs).is_integer(): 
        num_rows, num_cols = int(math.sqrt(num_graphs)), int(math.sqrt(num_graphs))
    elif num_graphs<=3:
        num_rows, num_cols = 1, 3
    elif num_graphs<=12:
        num_rows, num_cols = int(num_graphs/3) +1, 3
    else:
        num_rows, num_cols = int(num_graphs/4) +1, 4

    fig = plt.figure()
    gs = gridspec.GridSpec(num_rows, num_cols, height_ratios=[1] * num_rows, width_ratios=[1] * num_cols)

    #Ty/except is not the best way to do this, but allows this function to work for all the weird kinds of setup there is
    try:
        for i in range(len(names)):
            var_ax = plt.subplot(gs[i])
            if multiple_on_one_graph == True:
                for j in range(len(var)):
                    plot_graph(var[j][i], ts, tf, var_ax, legend[j], names[i])
            else:
                plot_graph(var[i], ts, tf, var_ax, legend[i], names[i])
    except:
        for i in range(len(names)):
            for v in range(len(var)):
                var_ax = plt.subplot(gs[i])
                plot_graph(var[v][i], ts, tf, var_ax, legend[v], names[i])
            if not sensors==None and "GPS" in sensors:
                if i == 0:
                    data = sensors["GPS"][:,0]
                    t = np.linspace(0, tf, len(data))
                    var_ax.scatter(t[0:len(data)], data, label = "GPS")
                    var_ax.legend()
                if i == 1:
                    data = sensors["GPS"][:,1]
                    t = np.linspace(0, tf, len(data))
                    var_ax.scatter(t[0:len(data)], data, label = "GPS")
                    var_ax.legend()
                if i == 2:
                    data = sensors["GPS"][:,2]
                    t = np.linspace(0, tf, len(data))
                    var_ax.scatter(t[0:len(data)], data, label = "GPS")
                    var_ax.legend()
                    data = sensors["Barometer"]
                    plot_graph(data, 1/BAROMETER_UPDATE_FREQ, tf, var_ax, "Barometer", names[i])
                if i == 3:
                    data = sensors["GPS"][:,3]
                    t = np.linspace(0, tf, len(data))
                    var_ax.scatter(t[0:len(data)], data, label = "GPS")
                    var_ax.legend()    
                    data = sensors["IMU"][:,0]
                    plot_graph(data, 1/ACCELEROMETER_UPDATE_FREQ, tf, var_ax, "IMU", names[i])                
                if i == 4:
                    data = sensors["GPS"][:,4]
                    t = np.linspace(0, tf, len(data))
                    var_ax.scatter(t[0:len(data)], data, label = "GPS")
                    var_ax.legend()    
                    data = sensors["IMU"][:,1]
                    plot_graph(data, 1/ACCELEROMETER_UPDATE_FREQ, tf, var_ax, "IMU", names[i])   
                if i == 5:
                    data = sensors["GPS"][:,5]
                    t = np.linspace(0, tf, len(data))
                    var_ax.scatter(t[0:len(data)], data, label = "GPS")
                    var_ax.legend()    
                    data = sensors["IMU"][:,2]
                    plot_graph(data, 1/ACCELEROMETER_UPDATE_FREQ, tf, var_ax, "IMU", names[i])                                 
            if not sensors==None and "Gyro" in sensors:    
                if i == 0:
                    data = sensors["Magnetometer"][:,0]
                    plot_graph(data, 1/MAGNETOMETER_UPDATE_FREQ, tf, var_ax, "Magnetometer", names[i])
                if i == 1:
                    data = sensors["Magnetometer"][:,1]
                    plot_graph(data, 1/MAGNETOMETER_UPDATE_FREQ, tf, var_ax, "Magnetometer", names[i])  
                # if i == 2:
                #     data = sensors["Magnetometer"][:,2]
                #     plot_graph(data, 1/MAGNETOMETER_UPDATE_FREQ, tf, var_ax, "Magnetometer", names[i])
                # if i == 3:
                #     data = sensors["Gyro"][:,0]
                #     plot_graph(data, 1/GYROSCOPE_UPDATE_FREQ, tf, var_ax, "Gyro", names[i])             
                # if i == 4:
                #     data = sensors["Gyro"][:,1]
                #     plot_graph(data, 1/GYROSCOPE_UPDATE_FREQ, tf, var_ax, "Gyro", names[i])        
                # if i == 5:
                #     data = sensors["Gyro"][:,2]
                #     plot_graph(data, 1/GYROSCOPE_UPDATE_FREQ, tf, var_ax, "Gyro", names[i])                

    plt.subplots_adjust(wspace=0.5, hspace=0.5)

    canvas = FigureCanvasTkAgg(fig, master=tab)
    canvas.draw()
    canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    # Add the toolbar below the canvas
    toolbar = NavigationToolbar2Tk(canvas, tab)
    toolbar.update()
    canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

def plot_graph(var, ts, tf, ax, legend, name='INSERT NAME HERE'):

    t = np.linspace(0, tf, len(var))
    ax.plot(t[0:len(var)], var, label = legend)

    ax.set_title("%s vs Time"%name)
    ax.set_xlabel("Time")
    ax.set_ylabel(name)
    ax.legend()

def plot_landing_graph(tab, title, xdata, ydata, xlabel, ylabel):
    # Create a Matplotlib figure and axis
    fig, ax = plt.subplots()
    ax.scatter(xdata, ydata)
    
    circle = plt.Circle((0, 0), MAX_RADIUS_ERROR, color='red', fill=False)
    plt.gca().add_patch(circle)
    

    # Set title and labels
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)

    # Embed Matplotlib figure in Tkinter window
    canvas = FigureCanvasTkAgg(fig, master=tab)
    canvas.draw()
    canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    # Add the toolbar below the canvas
    toolbar = NavigationToolbar2Tk(canvas, tab)
    toolbar.update()
    canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

def plot_time_series(time_vector, output_vector, clear_plot=False):
    """
    Plots a time series given a time vector and an output vector.

    Parameters:
        time_vector (list or numpy array): Time vector.
        output_vector (list or numpy array): Output vector.
        clear_plot (bool): Whether to clear the existing plot before plotting new data.
    """
    if clear_plot:
        plt.clf()

    plt.plot(time_vector, output_vector)
    plt.xlabel('Time')
    plt.ylabel('Output')
    plt.title('Time Series Plot')
    plt.grid(True)

def create_gui(sim, planned_trajectory, trajectory, ts, tf):
    
    #Set up tkinter
    root = tk.Tk()
    root.title("Simulation Data")
    width = root.winfo_screenwidth()
    height = root.winfo_screenheight()
    root.geometry("%dx%d" % (width, height))
    style = ttk.Style()
    style.configure("Custom.TNotebook", tabposition='wn', expand=1.25)
    style.configure("Custom.TNotebook.Tab", font=('Helvetica', 20))
    notebook = ttk.Notebook(root, style="Custom.TNotebook")
    notebook.pack(expand=1.25, fill="both", padx=20, pady=20)

    #Get data
    true_dynamics = pull_dynamics(sim.rocket.state_history, ts, tf)
    ffc_dynamics = pull_dynamics(sim.rocket.ffc.state_history, ts, tf)
    sensed_positional_dynamics, sensed_rotational_dynamics = pull_sensed_dynamics(sim.rocket.ffc.sensed_state_history, ts, tf)
    kalman_dynamics = pull_dynamics(sim.rocket.ffc.state_history, ts, tf)

    # Altitude vs Time
    tab0 = ttk.Frame(notebook)
    legend = ["Truth", "ffc", "ideal"]
    create_graph_set(tab0, [[sim.rocket.state_history[:,2]], [sim.rocket.ffc.state_history[:,2]], [sim.ideal_trajectory[:,2]]], ts, tf, ["Altitude"], 1, legend, multiple_on_one_graph=True)
    notebook.add(tab0, text="Altitude")

    # Position Error
    tab1 = ttk.Frame(notebook)
    position_error = [sim.rocket.error_history[:,0], sim.rocket.error_history[:,1], sim.rocket.error_history[:,2]]
    ffc_position_error = [sim.rocket.ffc.error_history[:,0], sim.rocket.ffc.error_history[:,1], sim.rocket.ffc.error_history[:,2]]
    error_names = ["X Error (m)", "Y Error (m)", "Z Error (m)"]
    legend = ["Truth", "ffc"]
    create_graph_set(tab1, [position_error, ffc_position_error], ts, tf, error_names, 3, legend, multiple_on_one_graph=True)
    notebook.add(tab1, text="Position Error")

    # Rotation Error
    tab2 = ttk.Frame(notebook)
    rotation_error = [sim.rocket.error_history[:,6] * RAD2DEG, sim.rocket.error_history[:,7] * RAD2DEG, sim.rocket.error_history[:,8] * RAD2DEG]
    ffc_rotation_error = [sim.rocket.ffc.error_history[:,6] * RAD2DEG, sim.rocket.ffc.error_history[:,7] * RAD2DEG, sim.rocket.ffc.error_history[:,8] * RAD2DEG]
    rot_error_names = ["Pitch Error (degrees)", "Yaw Error (degrees)", "Roll Error (degrees)"]
    legend = ["Truth", "Truth", "Truth"]
    create_graph_set(tab2, [rotation_error, ffc_rotation_error], ts, tf, rot_error_names, 3, legend, multiple_on_one_graph=True)
    notebook.add(tab2, text="Rotation Error")
    
    # Controls
    tab3 = ttk.Frame(notebook)
    gimbal_theta = np.arctan2(sim.rocket.engine.posy_history, sim.rocket.engine.posx_history) * RAD2DEG
    gimbal_psi = np.arctan2(np.sqrt((sim.rocket.engine.posx_history ** 2) + (sim.rocket.engine.posy_history ** 2)), sim.rocket.engine.length) * RAD2DEG
    controls = [gimbal_psi, gimbal_theta, sim.rocket.engine.throttle_history]
    control_names = ["Gimbal Psi (degrees)", "Gimbal Theta (degrees)", "Throttle (percent)"]
    legend = ["Truth", "Truth", "Truth"]
    create_graph_set(tab3, controls, ts, tf, control_names, 3, legend)
    notebook.add(tab3, text="Gimbal Angles")
    
        # Controls 2
    tab8 = ttk.Frame(notebook)
    controls = [sim.rocket.engine.posx_history, sim.rocket.engine.posy_history, sim.rocket.engine.throttle_history]
    control_names = ["PosX (m)", "PosY (m)", "Throttle (percent)"]
    legend = ["Truth", "Truth", "Truth"]
    create_graph_set(tab8, controls, ts, tf, control_names, 3, legend)
    notebook.add(tab8, text="Gimbal Positions")
    
    # Controls 3
    tab12 = ttk.Frame(notebook)
    controls = [sim.rocket.ffc.u_history[:,0], sim.rocket.ffc.u_history[:,1], sim.rocket.ffc.u_history[:,2]]
    control_names = ["Ux (m/s^2)", "Uy (m/s^2)", "Uz (m/s^2)"]
    legend = ["ffc", "ffc", "ffc"]
    create_graph_set(tab12, controls, ts, tf, control_names, 3, legend)
    notebook.add(tab12, text="Control Inputs (U)")
    
    # MOI
    tab4 = ttk.Frame(notebook)
    moi = [[arr[0,0] for arr in sim.rocket.I_history], [arr[1,1] for arr in sim.rocket.I_history], [arr[2,2] for arr in sim.rocket.I_history]]
    moi_names = ["Ixx (kgm2)", "Iyy (kgm2)", "Izz (kgm2)"]
    legend = ["Truth", "Truth", "Truth"]
    create_graph_set(tab4, moi, ts, tf, moi_names, 3, legend)
    notebook.add(tab4, text="MOI")

    # Dynamics
    tab5 = ttk.Frame(notebook)
    dynamics_plot_names = ["X Position (m)", "Y Position (m)", "Z Position (m)", 
                           "X Velocity (m/s)", "Y Velocity (m/s)", "Z Velocity (m/s)", 
                           "X Acceleration (m/s2)", "Y Acceleration (m/s2)", "Z Acceleration (m/s2)"]
    legend = ["Truth", "ffc"]
    create_graph_set(tab5, [true_dynamics[0:9], ffc_dynamics[0:9]], ts, tf, dynamics_plot_names, 9, legend)
    notebook.add(tab5, text="Dynamics")

    # Rotational Dynamics
    tab6 = ttk.Frame(notebook)
    rotational_dynamics_plot_names = ["Pitch (degrees)", "Yaw (degrees)", "Roll (degrees)", 
                                      "Wx (deg/s)", "Wy (deg/s)", "Wz (deg/s)", 
                                      "Pitch Acceleration (deg/s2)", "Yaw Acceleration (deg/s2)", "Roll Acceleration (deg/s2)"]
    legend = ["Truth","ffc"]
    create_graph_set(tab6, [[x * RAD2DEG for x in true_dynamics[9:18]], [x * RAD2DEG for x in ffc_dynamics[9:18]]], ts, tf, rotational_dynamics_plot_names, 9, legend)
    notebook.add(tab6, text="Rotational Dynamics")
    
    # Wind
    tab7 = ttk.Frame(notebook)
    rotational_dynamics_plot_names = ["Pitch", "Yaw", "Roll", "Pitch Rate", "Yaw Rate", "Roll Rate", "Pitch Acceleration", "Yaw Acceleration", "Roll Acceleration"]
    legend = ["Truth"]
    create_graph_set(tab7, [np.linalg.norm(sim.wind_history, axis=1), sim.rocket.engine.posx_history * 10, sim.rocket.engine.posy_history * 10, sim.rocket.engine.throttle_history], ts, tf, ["Wind"], 1, legend)
    notebook.add(tab7, text="Wind")

    # Landing
    tab9 = ttk.Frame(notebook)
    if sim.landed == True:
        plot_landing_graph(tab9, "Landing Position", sim.rocket.error_history[-1,0], sim.rocket.error_history[-1, 1], "X Position (m)", "Y Position (m)")
        notebook.add(tab9, text="Landing")
        
    # Sensors
    tab10 = ttk.Frame(notebook)
    dynamics_plot_names = ["X Position (m)", "Y Position (m)", "Z Position (m)", 
                           "X Velocity (m/s)", "Y Velocity (m/s)", "Z Velocity (m/s)", 
                           "X Acceleration (m/s2)", "Y Acceleration (m/s2)", "Z Acceleration (m/s2)"]
    legend = ["T", "K"]

    create_graph_set(tab10, [true_dynamics[0:9], kalman_dynamics[0:9]], ts, tf, dynamics_plot_names, 9, legend, sensors=sensed_positional_dynamics)
    notebook.add(tab10, text="Sensed Dynamics")
    
    # Sensors
    tab11 = ttk.Frame(notebook)
    dynamics_plot_names = ["Pitch (degrees)", "Yaw (degrees)", "Roll (degrees)", 
                                      "Pitch Rate (deg/s)", "Yaw Rate (deg/s)", "Roll Rate (deg/s)", 
                                      "Pitch Acceleration (deg/s2)", "Yaw Acceleration (deg/s2)", "Roll Acceleration (deg/s2)"]
    legend = ["T", "K"]
    create_graph_set(tab11, [[x * RAD2DEG for x in true_dynamics[9:18]], [x * RAD2DEG for x in kalman_dynamics[9:18]]], ts, tf, dynamics_plot_names, 9, legend, 
                     sensors=sensed_rotational_dynamics)
    notebook.add(tab11, text="Sensed Rotations")
    
    '''
    To add new graphs use the following format:

    tabn = ttk.Frame(notebook)
    names = ["list of all the names you want the graphs to be titled, one for each graph"]
    create_graph_set(tabn, the data itself, ts, tf, names, number of graphs)
    notebook.add(tabn, text="| tab title |")
    '''              
    root.mainloop()