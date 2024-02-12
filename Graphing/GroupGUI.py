import numpy as np
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from Simulator.simulationConstants import *

def pull_dynamics(trajectory, ts, tf):
    frame=len(trajectory)
    # reads x, y, z positions
    x_pos = trajectory[:frame, 0]
    y_pos = trajectory[:frame, 1]
    z_pos = trajectory[:frame, 2]
    theta_x = trajectory[:frame, 6] * RAD2DEG
    theta_y = trajectory[:frame, 7] * RAD2DEG
    theta_z = trajectory[:frame, 8] * RAD2DEG

    # reads x, y, z velocity
    x_vel = trajectory[:frame, 3]
    y_vel = trajectory[:frame, 4]
    z_vel = trajectory[:frame, 5]
    omega_x = trajectory[:frame, 9] * RAD2DEG
    omega_y = trajectory[:frame, 10] * RAD2DEG
    omega_z = trajectory[:frame, 11] * RAD2DEG

    t = np.linspace(0, tf, int(tf/ts)+1)

    # differentiates velocity for x, y, z acceleration
    x_acc = np.diff(x_vel) / np.diff(t[0:len(x_vel)])
    y_acc = np.diff(y_vel) / np.diff(t[0:len(y_vel)])
    z_acc = np.diff(z_vel) / np.diff(t[0:len(z_vel)])
    x_alpha = (np.diff(omega_x) / np.diff(t[0:len(omega_x)])) * RAD2DEG
    y_alpha = (np.diff(omega_y) / np.diff(t[0:len(omega_y)])) * RAD2DEG
    z_alpha = (np.diff(omega_z) / np.diff(t[0:len(omega_z)])) * RAD2DEG

    return [x_pos, y_pos, z_pos, x_vel, y_vel, z_vel, x_acc, y_acc, z_acc, theta_x, theta_y, theta_z, omega_x, omega_y, omega_z, x_alpha, y_alpha, z_alpha]

def plot_dynamics(tab, dynamic_vars, ts, tf, names=None):
    if names is None:
        names = ["INSERT NAME HERE" for i in range(len(dynamic_vars[0]))]
    nrows, ncols = 3, 3
    fig = plt.figure()
    gs = gridspec.GridSpec(nrows, ncols, height_ratios=[1, 1, 1], width_ratios=[1, 1, 1])
    for var in dynamic_vars:
        for i in range(len(var)):
            var_ax = plt.subplot(gs[i])
            plot_variable_vs_time_on_subplot(var[i], ts, tf, var_ax, names[i])

    plt.subplots_adjust(wspace=0.5, hspace=0.5)
    
    # Embed Matplotlib figure in Tkinter window
    canvas = FigureCanvasTkAgg(fig, master=tab)
    canvas.draw()
    canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

def plot_variable_vs_time_on_subplot(var, ts, tf, ax, name='INSERT NAME HERE'):

    t = np.linspace(0, tf, int(tf/ts)+1)

    ax.plot(t[0:len(var)], var)

    ax.set_title("%s vs Time"%name)
    ax.set_xlabel("Time")
    ax.set_ylabel(name)

def plot_variables_vs_time(tab, vars, ts, tf, name='INSERT NAME HERE'):
    fig, ax = plt.subplots()

    t = np.linspace(0, tf, int(tf/ts)+1)

    for var in vars:
        ax.plot(t[0:len(var)], var)


    ax.set_title("%s vs Time"%name)
    ax.set_xlabel("Time")
    ax.set_ylabel(name)

    # Embed Matplotlib figure in Tkinter window
    canvas = FigureCanvasTkAgg(fig, master=tab)
    canvas.draw()
    canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
    
def create_3_graph(tab, vars, ts, tf, names, title):
    nrows, ncols = 1, 3
    fig, axs = plt.subplots(nrows, ncols)
    for i in range(3):
        var_ax = axs[i%ncols]
        for var in vars[i]:
            plot_variable_vs_time_on_subplot(var, ts, tf, var_ax, names[i])
    
    # Embed Matplotlib figure in Tkinter window
    canvas = FigureCanvasTkAgg(fig, master=tab)
    canvas.draw()
    canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

def landing_graph(tab, title, xdata, ydata, xlabel, ylabel, num_suc, num_total):
    # Create a Matplotlib figure and axis
    fig, ax = plt.subplots()
    ax.scatter(xdata, ydata)
    
    circle = plt.Circle((0, 0), MAX_RADIUS_ERROR, color='red', fill=False)
    plt.gca().add_patch(circle)
    

    # Set title and labels
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)

    plt.text(MAX_RADIUS_ERROR - 0.2, MAX_RADIUS_ERROR - 0.2, '{} Landed out of {}'.format(num_suc, num_total), fontsize=12, color='black')
    # Embed Matplotlib figure in Tkinter window
    canvas = FigureCanvasTkAgg(fig, master=tab)
    canvas.draw()
    canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

def create_group_gui(sims, planned_trajectory, trajectories, ts, tf):
    root = tk.Tk()
    root.title("Simulation Data")

    style = ttk.Style()
    style.configure("TNotebook.Tab", font=('Helvetica', 20))  # Adjust font size here

    notebook = ttk.Notebook(root)

    # Altitude vs Time
    tab0 = ttk.Frame(notebook)
    plot_variables_vs_time(tab0, [planned_trajectory[:,2]] + [trajectory[:,2] for trajectory in trajectories], ts, tf, name="Altitude")
    notebook.add(tab0, text="| ALTITUDE |")

    # Position Error
    position_error = [[sim.error_history[:,0] for sim in sims if sim.landed == True], [sim.error_history[:,1] for sim in sims if sim.landed == True], [sim.error_history[:,2] for sim in sims if sim.landed == True]]
    error_names = ["X Error (m)", "Y Error (m)", "Z Error (m)"]
    tab1 = ttk.Frame(notebook)
    create_3_graph(tab1, position_error, ts, tf, error_names, "Position Error")
    notebook.add(tab1, text="| Position Error |")

    # # Rotation Error
    rotation_error = [[sim.error_history[:,0] * RAD2DEG for sim in sims if sim.landed == True], [sim.error_history[:,1] * RAD2DEG for sim in sims if sim.landed == True], [sim.error_history[:,2] * RAD2DEG for sim in sims if sim.landed == True]]
    rot_error_names = ["Pitch Error (degrees)", "Yaw Error (degrees)", "Roll Error (degrees)"]
    tab2 = ttk.Frame(notebook)
    create_3_graph(tab2, rotation_error, ts, tf, rot_error_names, "Rotation Error")
    notebook.add(tab2, text="| Rotation Error |")
    
    # Controls
    gimbal_theta = []
    gimbal_psi = []
    for sim in sims:
        gimbal_theta.append(np.arctan2(sim.rocket.engine.posy_history, sim.rocket.engine.posx_history) * RAD2DEG)
        gimbal_psi.append(np.arctan2(np.sqrt((sim.rocket.engine.posx_history ** 2) + (sim.rocket.engine.posy_history ** 2)), sim.rocket.engine.length) * RAD2DEG)
    controls = [gimbal_psi, gimbal_theta, [sim.rocket.engine.throttle_history for sim in sims]]
    control_names = ["Gimbal Psi (degrees)", "Gimbal Theta (degrees)", "Throttle (percent)"]
    tab3 = ttk.Frame(notebook)
    create_3_graph(tab3, controls, ts, tf, control_names, "Control Inputs")
    notebook.add(tab3, text="| Control Inputs |")
    
    # Controls 2
    controls = [[sim.rocket.engine.posx_history for sim in sims], [sim.rocket.engine.posy_history for sim in sims], [sim.rocket.engine.throttle_history for sim in sims]]
    control_names = ["PosX (m)", "PosY (m)", "Throttle (percent)"]
    tab8 = ttk.Frame(notebook)
    create_3_graph(tab8, controls, ts, tf, control_names, "Control Inputs")
    notebook.add(tab8, text="| Control Inputs |")
    
    # Dynamics
    dynamics = [pull_dynamics(trajectory, ts, tf)[0:9] for trajectory in trajectories]
    dynamics_plot_names = ["X Position (m)", "Y Position (m)", "Z Position (m)", 
                           "X Velocity (m/s)", "Y Velocity (m/s)", "Z Velocity (m/s)", 
                           "X Acceleration (m/s2)", "Y Acceleration (m/s2)", "Z Acceleration (m/s2)"]
    tab5 = ttk.Frame(notebook)
    plot_dynamics(tab5, dynamics, ts, tf, dynamics_plot_names)
    notebook.add(tab5, text="| Dynamics |")

    # Rotational Dynamics
    rot_dynamics = [pull_dynamics(trajectory, ts, tf)[9:18] for trajectory in trajectories]
    rotational_dynamics_plot_names = ["Pitch (degrees)", "Yaw (degrees)", "Roll (degrees)", 
                                      "Pitch Rate (deg/s)", "Yaw Rate (deg/s)", "Roll Rate (deg/s)", 
                                      "Pitch Acceleration (deg/s2)", "Yaw Acceleration (deg/s2)", "Roll Acceleration (deg/s2)"]
    tab6 = ttk.Frame(notebook)
    plot_dynamics(tab6, rot_dynamics, ts, tf, rotational_dynamics_plot_names)
    notebook.add(tab6, text="| Rotational Dynamics |")
    
    # Wind
    rotational_dynamics_plot_names = ["Pitch", "Yaw", "Roll", "Pitch Rate", "Yaw Rate", "Roll Rate", "Pitch Acceleration", "Yaw Acceleration", "Roll Acceleration"]
    tab7 = ttk.Frame(notebook)
    plot_variables_vs_time(tab7, [np.linalg.norm(sim.wind_history, axis=1) for sim in sims], ts, tf, "Wind")
    notebook.add(tab7, text="| Wind |")
    
    # Landing
    tab9 = ttk.Frame(notebook)
    num_landed = 0
    for sim in sims:
        if sim.landed==True:
            num_landed+=1
    landing_graph(tab9, "Landing Position", [sim.error_history[-1,0] for sim in sims if sim.landed==True], [sim.error_history[-1, 1] for sim in sims if sim.landed==True], "X Position (m)", "Y Position (m)", num_landed, len(sims))
    notebook.add(tab9, text="| Landing |")
    
    # U
    tab10 = ttk.Frame(notebook)
    Us = [[sim.u_history[:,0] for sim in sims if sim.landed == True], [sim.u_history[:,1] for sim in sims if sim.landed == True], [sim.u_history[:,2] for sim in sims if sim.landed == True]]
    control_names = ["UX", "UY", "UZ"]
    for sim in sims:
        if sim.landed==True:
            num_landed+=1
    create_3_graph(tab10, Us, ts, tf, control_names, "Control Inputs")
    notebook.add(tab10, text="| U |")
    
    
    notebook.pack(expand=1.25, fill="both", padx=10, pady=10)
    root.mainloop()
