# Run this app with `python app.py` and
# visit http://127.0.0.1:8050/ in your web browser.


from dash import Dash, html, dcc
import plotly.express as px
import plotly.graph_objs as go
import pandas as pd

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
    x_acc = np.diff(x_vel) / np.diff(t[0:len(x_vel)])
    y_acc = np.diff(y_vel) / np.diff(t[0:len(y_vel)])
    z_acc = np.diff(z_vel) / np.diff(t[0:len(z_vel)])
    x_alpha = np.diff(omega_x) / np.diff(t[0:len(omega_x)])
    y_alpha = np.diff(omega_y) / np.diff(t[0:len(omega_y)])
    z_alpha = np.diff(omega_z) / np.diff(t[0:len(omega_z)])
    
    x_acc = np.append(x_acc, x_acc[-1])
    y_acc = np.append(y_acc, y_acc[-1])
    z_acc = np.append(z_acc, z_acc[-1])
    x_alpha = np.append(x_alpha, x_alpha[-1])
    y_alpha = np.append(y_alpha, y_alpha[-1])
    z_alpha = np.append(z_alpha, z_alpha[-1])

    return [x_pos, y_pos, z_pos, x_vel, y_vel, z_vel, x_acc, y_acc, z_acc, theta_x, theta_y, theta_z, omega_x, omega_y, omega_z, x_alpha, y_alpha, z_alpha]

def run_dash(sim, planned_trajectory, trajectory, ts, tf):
    app = Dash(__name__)

    # assume you have a "long-form" data frame
    # see https://plotly.com/python/px-arguments/ for more options0

    t_length = len(sim.rocket.state_history[:, 2])
    true_dynamics = pull_dynamics(sim.rocket.state_history, ts, tf)
    ffc_dynamics = pull_dynamics(sim.rocket.ffc.state_history, ts, tf)
    kalman_dynamics = pull_dynamics(sim.rocket.ffc.state_history, ts, tf)
    def pull_sensed_dynamics(sensed_state_history, ts, tf):
        positional = {"GPS": sensed_state_history[:,0:6],
                    "Barometer": sensed_state_history[:,6],
                    "IMU": sensed_state_history[:,7:10]}
        rotational = {"Magnetometer": sensed_state_history[:,10:13] * RAD2DEG,
                    "Gyro": sensed_state_history[:,13:16] * RAD2DEG}
        return positional, rotational
    sensed_positional_dynamics, sensed_rotational_dynamics = pull_sensed_dynamics(sim.rocket.ffc.sensed_state_history, ts, tf)
    data = pd.DataFrame(
        {
            "time": np.linspace(0, tf, t_length),
            "Altitude History": sim.rocket.state_history[:, 2],
            "Altitude FFC History": sim.rocket.ffc.state_history[:, 2],
            "X Truth position_error": sim.rocket.error_history[:, 0],
            "Y Truth position_error": sim.rocket.error_history[:, 1],
            "Z Truth position_error": sim.rocket.error_history[:, 2],
            "X FFC position_error": sim.rocket.ffc.error_history[:, 0],
            "Y FFC position_error": sim.rocket.ffc.error_history[:, 1],
            "Z FFC position_error": sim.rocket.ffc.error_history[:, 2],
            "Pitch Truth Error": sim.rocket.error_history[:,6] * RAD2DEG,
            "Yaw Truth Error": sim.rocket.error_history[:,7] * RAD2DEG,
            "Roll Truth Error": sim.rocket.error_history[:,8] * RAD2DEG,
            "Pitch FFC Error": sim.rocket.ffc.error_history[:,6] * RAD2DEG,
            "Yaw FFC Error": sim.rocket.ffc.error_history[:,7] * RAD2DEG,
            "Roll FFC Error": sim.rocket.ffc.error_history[:,8] * RAD2DEG,
            "Gimbal Theta": np.arctan2(sim.rocket.engine.posy_history, sim.rocket.engine.posx_history) * RAD2DEG,
            "Gimbal Psi": np.arctan2(np.sqrt((sim.rocket.engine.posx_history ** 2) + (sim.rocket.engine.posy_history ** 2)), sim.rocket.engine.length) * RAD2DEG,
            "Throttle": sim.rocket.engine.throttle_history,
            "Gimbal X Position": sim.rocket.engine.posx_history,
            "Gimbal Y Position":sim.rocket.engine.posy_history,
            "Control Input X": sim.rocket.ffc.u_history[:,0],
            "Control Input Y": sim.rocket.ffc.u_history[:,1],
            "Control Input Z": sim.rocket.ffc.u_history[:,2],
            "Moment of Inertia XX": [arr[0,0] for arr in sim.rocket.I_history],
            "Moment of Inertia YY": [arr[1,1] for arr in sim.rocket.I_history],
            "Moment of Inertia ZZ": [arr[2,2] for arr in sim.rocket.I_history],
            "X Truth": true_dynamics[0],
            "Y Truth": true_dynamics[1],
            "Z Truth": true_dynamics[2],
            "Vx Truth": true_dynamics[3],
            "Vy Truth": true_dynamics[4],
            "Vz Truth": true_dynamics[5],
            "Ax Truth": true_dynamics[6],
            "Ay Truth": true_dynamics[7],
            "Az Truth": true_dynamics[8],
            "Pitch Truth": true_dynamics[9],
            "Yaw Truth": true_dynamics[10],
            "Roll Truth": true_dynamics[11],
            "Omega Pitch Truth": true_dynamics[12],
            "Omega Yaw Truth": true_dynamics[13],
            "Omega Roll Truth": true_dynamics[14],
            "Alpha Pitch Truth": true_dynamics[15],
            "Alpha Yaw Truth": true_dynamics[16],
            "Alpha Roll Truth": true_dynamics[17],
            "X FFC": ffc_dynamics[0],
            "Y FFC": ffc_dynamics[1],
            "Z FFC": ffc_dynamics[2],
            "Vx FFC": ffc_dynamics[3],
            "Vy FFC": ffc_dynamics[4],
            "Vz FFC": ffc_dynamics[5],
            "Ax FFC": ffc_dynamics[6],
            "Ay FFC": ffc_dynamics[7],
            "Az FFC": ffc_dynamics[8],
            "Pitch FFC": ffc_dynamics[9],
            "Yaw FFC": ffc_dynamics[10],
            "Roll FFC": ffc_dynamics[11],
            "Omega Pitch FFC": ffc_dynamics[12],
            "Omega Yaw FFC": ffc_dynamics[13],
            "Omega Roll FFC": ffc_dynamics[14],
            "Alpha Pitch FFC": ffc_dynamics[15],
            "Alpha Yaw FFC": ffc_dynamics[16],
            "Alpha Roll FFC": ffc_dynamics[17],
            "X Kalman": kalman_dynamics[0],
            "Y Kalman": kalman_dynamics[1],
            "Z Kalman": kalman_dynamics[2],
            "Vx Kalman": kalman_dynamics[3],
            "Vy Kalman": kalman_dynamics[4],
            "Vz Kalman": kalman_dynamics[5],
            "Ax Kalman": kalman_dynamics[6],
            "Ay Kalman": kalman_dynamics[7],
            "Az Kalman": kalman_dynamics[8],
            "Pitch Kalman": kalman_dynamics[9],
            "Yaw Kalman": kalman_dynamics[10],
            "Roll Kalman": kalman_dynamics[11],
            "Omega Pitch Kalman": kalman_dynamics[12],
            "Omega Yaw Kalman": kalman_dynamics[13],
            "Omega Roll Kalman": kalman_dynamics[14],
            "Alpha Pitch Kalman": kalman_dynamics[15],
            "Alpha Yaw Kalman": kalman_dynamics[16],
            "Alpha Roll Kalman": kalman_dynamics[17],
            "X GPS": sensed_positional_dynamics["GPS"][:,0],
            "Y GPS": sensed_positional_dynamics["GPS"][:,1],
            "Z GPS": sensed_positional_dynamics["GPS"][:,2],
            "Vx GPS": sensed_positional_dynamics["GPS"][:,3],
            "Vy GPS": sensed_positional_dynamics["GPS"][:,4],
            "Vz GPS": sensed_positional_dynamics["GPS"][:,5],
            "Barometer": sensed_positional_dynamics["Barometer"],
            "IMU X": sensed_positional_dynamics["IMU"][:,0],
            "IMU Y": sensed_positional_dynamics["IMU"][:,1],
            "IMU Z": sensed_positional_dynamics["IMU"][:,2],
            "Magnetometer Pitch": sensed_rotational_dynamics["Magnetometer"][:,0],
            "Magnetometer Yaw": sensed_rotational_dynamics["Magnetometer"][:,1],
            "Magnetometer Roll": sensed_rotational_dynamics["Magnetometer"][:,2], 
            "Gyro Pitch": sensed_rotational_dynamics["Gyro"][:,0],
            "Gyro Yaw": sensed_rotational_dynamics["Gyro"][:,1],
            "Gyro Roll": sensed_rotational_dynamics["Gyro"][:,2],

        }
    )
    #ALTITUDE:
    altitude = px.line(
        data,
        x="time",
        y=["Altitude History", "Altitude FFC History"],
        title="Altitude",
        labels={
            'value': 'Altitude',
        }
    )

    #POSITION ERROR
    position_x = px.line(
        data,
        x="time",
        y=["X Truth position_error", "X FFC position_error"],
        title="X Position Error",
        labels={
            'value': 'X Error (m)', 'time':'Time'
        }
    )
    position_y = px.line(
        data,
        x="time",
        y=["Y Truth position_error", "Y FFC position_error"],
        title="Y Position Error",
        labels={
            'value': 'Y Error (m)', 'time':'Time'
        }
    )
    position_z = px.line(
        data,
        x="time",
        y=["Z Truth position_error", "Z FFC position_error"],
        title="Z Position Error",
        labels={
            'value': 'Z Error (m)', 'time':'Time'
        }
    )

    #ROTATION ERROR:
    pitch_error = px.line(
        data,
        x="time",
        y = ["Pitch Truth Error", "Pitch FFC Error"],
        title = "Pitch Position Error",
        labels={
            'value': 'Pitch Error (degrees)', 'time':'Time'
        }
    )

    roll_error = px.line(
        data,
        x="time",
        y = ["Roll Truth Error", "Roll FFC Error"],
        title = "Roll Position Error",
        labels={
            'value': 'Roll Error (degrees)', 'time':'Time'
        }
    )

    yaw_error = px.line(
        data,
        x="time",
        y = ["Yaw Truth Error", "Yaw FFC Error"],
        title = "Yaw Position Error",
        labels={
            'value': 'Yaw Error (degrees)', 'time':'Time'
        }
    )

    #GIMBAL ANGLES:
    gimbal_psi = px.line(
        data,
        x="time",
        y = ["Gimbal Psi"],
        title = "Gimbal Psi (Degrees)",
        labels={
            'value': 'Gimbal PSI (Degrees)', 'time':'Time'
        }
    )

    gimbal_theta = px.line(
        data,
        x="time",
        y = ["Gimbal Theta"],
        title = "Gimbal Theta (Degrees)",
        labels={
            'value': 'Gimbal Theta (Degrees)', 'time':'Time'
        }
    )

    throttle = px.line(
        data,
        x="time",
        y = ["Throttle"],
        title = "Throttle Percent",
        labels={
            'value': 'Throttle (Percent)', 'time':'Time'
        }
    )

    #GIMBAL POSITIONS:
    posX = px.line(
        data,
        x="time",
        y = ["Gimbal X Position"],
        title = "PosX (m) vs. Time",
        labels={
            'value': 'PosX (m)', 'time':'Time'
        }
    )

    posY = px.line(
        data,
        x="time",
        y = ["Gimbal Y Position"],
        title = "PosY (m) vs. Time",
        labels={
            'value': 'PosY (m)', 'time':'Time'
        }
    )

    #CONTROL INPUTS (U)
    ux = px.line(
        data,
        x = 'time',
        y = ['Control Input X'],
        title = 'Ux (m/s^2 vs Time)',
        labels={
            'value': 'Ux (m/s^2)', 'time':'Time'
        }
    )

    uy = px.line(
        data,
        x = 'time',
        y = ['Control Input Y'],
        title = 'Yx (m/s^2 vs Time)',
        labels={
            'value': 'Uy (m/s^2)', 'time':'Time'
        }
    )

    uz = px.line(
        data,
        x = 'time',
        y = ['Control Input Z'],
        title = 'Zx (m/s^2 vs Time)',
        labels={
            'value': 'Uz (m/s^2)', 'time':'Time'
        }
    )

    #MOI: 
    moixx = px.line(
        data,
        x = 'time',
        y = ['Moment of Inertia XX'],
        title = 'IXX (kgm2) vs Time',
        labels={
            'value': 'Ixx (kgm^2)', 'time':'Time'
        }
    )
    moiyy = px.line(
        data,
        x = 'time',
        y = ['Moment of Inertia YY'],
        title = 'IYY (kgm2) vs Time',
        labels={
            'value': 'Iyy (kgm^2)', 'time':'Time'
        }
    )
    moizz = px.line(
        data,
        x = 'time',
        y = ['Moment of Inertia ZZ'],
        title = 'IZZ (kgm2) vs Time',
        labels={
            'value': 'Izz (kgm^2)', 'time':'Time'
        }
    )

    #DYNAMICS:
    #position
    x_position = px.line(
        data,
        x = 'time',
        y = ['X FFC', 'X Truth'],
        title = 'X Positon (m) vs. Time',
        labels={
            'value': 'X Position (m)', 'time':'Time'
        }
    )
    y_position = px.line(
        data,
        x = 'time',
        y = ['Y FFC', 'Y Truth'],
        title = 'Y Positon (m) vs. Time',
        labels={
            'value': 'Y Position (m)', 'time':'Time'
        }
    )
    z_position = px.line(
        data,
        x = 'time',
        y = ['Z FFC', 'Z Truth'],
        title = 'Z Positon (m) vs. Time',
        labels={
            'value': 'Z Position (m)', 'time':'Time'
        }
    )

    #velocity
    x_velocity = px.line(
        data,
        x = 'time',
        y = ['Vx FFC', 'Vx Truth'],
        title = 'X Velocity (m) vs. Time',
        labels={
            'value': 'X Velocity (m/s)', 'time':'Time'
        }
    )
    y_velocity = px.line(
        data,
        x = 'time',
        y = ['Vy FFC', 'Vy Truth'],
        title = 'Y Velocity (m) vs. Time',
        labels={
            'value': 'Y Velocity (m/s)', 'time':'Time'
        }
    )
    z_velocity = px.line(
        data,
        x = 'time',
        y = ['Vz FFC', 'Vz Truth'],
        title = 'Z Velocity (m) vs. Time',
        labels={
            'value': 'Z Velocity (m/s)', 'time':'Time'
        }
    )

    #acceleration
    x_acceleration = px.line(
        data,
        x = 'time',
        y = ['Ax FFC', 'Ax Truth'],
        title = 'X Acceleration (m) vs. Time',
        labels={
            'value': 'X Acceleration (m/s^2)', 'time':'Time'
        }
    )
    y_acceleration = px.line(
        data,
        x = 'time',
        y = ['Ay FFC', 'Ay Truth'],
        title = 'Y Acceleration (m) vs. Time',
        labels={
            'value': 'Y Acceleration (m/s^2)', 'time':'Time'
        }
    )
    z_acceleration = px.line(
        data,
        x = 'time',
        y = ['Az FFC', 'Az Truth'],
        title = 'Z Acceleration (m) vs. Time',
        labels={
            'value': 'Z Acceleration (m/s^2)', 'time':'Time'
        }
    )

    #ROTATIONAL DYNAMICS 
    #degrees
    pitch_degrees = px.line(
        data,
        x = 'time',
        y = ['Pitch FFC', 'Pitch Truth'],
        title = 'Pitch Degrees vs Time',
        labels={
            'value': 'Pitch (degrees)', 'time':'Time'
        }
    )
    yaw_degrees = px.line(
        data,
        x = 'time',
        y = ['Yaw FFC', 'Yaw Truth'],
        title = 'Yaw Degrees vs Time',
        labels={
            'value': 'Yaw (degrees)', 'time':'Time'
        }
    )
    roll_degrees = px.line(
        data,
        x = 'time',
        y = ['Roll FFC', 'Roll Truth'],
        title = 'Roll Degrees vs Time',
        labels={
            'value': 'Roll (degrees)', 'time':'Time'
        }
    )

    #omega
    pitch_omega = px.line(
        data,
        x = 'time',
        y = ['Omega Pitch FFC', 'Omega Pitch Truth'],
        title = 'Omega Pitch (deg/s) vs Time',
        labels={
            'value': 'Wx (deg/s)', 'time':'Time'
        }
    )
    yaw_omega = px.line(
        data,
        x = 'time',
        y = ['Omega Yaw FFC', 'Omega Yaw Truth'],
        title = 'Omega Yaw (deg/s) vs Time',
        labels={
            'value': 'Wy (deg/s)', 'time':'Time'
        }
    )
    roll_omega = px.line(
        data,
        x = 'time',
        y = ['Omega Roll FFC', 'Omega Roll Truth'],
        title = 'Omega Roll (deg/s) vs Time',
        labels={
            'value': 'Wz (deg/s)', 'time':'Time'
        }
    )

    #alpha
    pitch_alpha = px.line(
        data,
        x = 'time',
        y = ['Alpha Pitch FFC', 'Alpha Pitch Truth'],
        title = 'Alpha Pitch (deg/s^2) vs Time',
        labels={
            'value': 'Pitch Acceleration (deg/s^2)', 'time':'Time'
        }
    )
    yaw_alpha = px.line(
        data,
        x = 'time',
        y = ['Alpha Yaw FFC', 'Alpha Yaw Truth'],
        title = 'Alpha Yaw (deg/s^2) vs Time',
        labels={
            'value': 'Yaw Acceleration (deg/s^2)', 'time':'Time'
        }
    )
    roll_alpha = px.line(
        data,
        x = 'time',
        y = ['Alpha Roll FFC', 'Alpha Roll Truth'],
        title = 'Alpha Roll (deg/s^2) vs Time',
        labels={
            'value': 'Roll Acceleration (deg/s^2)', 'time':'Time'
        }
    )

    #WIND
    wind = px.line(
        data,
        x = 'time',
        y = [np.linalg.norm(sim.wind_history, axis=1)],
        title = 'Wind vs Time',
        labels={
            'value': 'Wind', 'time':'Time'
        }
    )

    #LANDING POSITION
    if sim.landed == True:
        landing_positon = px.scatter(
            data,
            x = [sim.rocket.error_history[-1,0]],
            y = [sim.rocket.error_history[-1,1]],
            title = 'Landing Postion',
            labels={
            'value': 'Y Position (m)', 'time':'X Position (m)'
        }
        )
    else:
        landing_positon = px.scatter(
        )
    landing_positon.add_shape(type="circle", x0 = -1, y0 = -1, x1 = 1, y1 = 1, line_color = 'red')

    #SENSED DYNAMICS
    #Positions
    sensed_dynamics_positon_x = px.scatter(
        data,
        x = 'time',
        y = ['X Truth', 'X Kalman', 'X GPS'],
        title = 'X positon (m) vs Time',
        labels={
            'value': 'X Position (m)', 'time':'Time'
        },
    )
    sensed_dynamics_positon_y = px.scatter(
        data,
        x = 'time',
        y = ['Y Truth', 'Y Kalman', 'Y GPS'],
        title = 'Y positon (m) vs Time',
        labels={
            'value': 'Y Positçion (m)', 'time':'Time'
        },
    )
    sensed_dynamics_positon_z = px.scatter(
        data,
        x = 'time',
        y = ['Z Truth', 'Z Kalman', 'Z GPS', 'Barometer'],
        title = 'Z positon (m) vs Time',
        labels={
            'value': 'Z Position (m)', 'time':'Time'
        },
    )

    sensed_dynamics_velocity_x = px.scatter(
        data,
        x = 'time',
        y = ["Vx Truth", "Vx Kalman", "Vx GPS", "IMU X"],
        title = "X Velocity (m/s) vs Time",
        labels={
            'value': 'X Velocity (m/s)', 'time':'Time'
        },

    )

    #Velocity
    sensed_dynamics_velocity_y = px.scatter(
        data,
        x = 'time',
        y = ["Vy Truth", "Vy Kalman", "Vy GPS", "IMU Y"],
        title = "Y Velocity (m/s) vs Time",
        labels={
            'value': 'Y Velocity (m/s)', 'time':'Time'
        },

    )

    sensed_dynamics_velocity_z = px.scatter(
        data,
        x = 'time',
        y = ["Vz Truth", "Vz Kalman", "Vz GPS", "IMU Z"],
        title = "Z Velocity (m/s) vs Time",
        labels={
            'value': 'Z Velocity (m/s)', 'time':'Time'
        },

    )

    #Accelerations
    sensed_dynamics_acceleration_x = px.line(
        data,
        x = 'time',
        y = ['Ax Truth', 'Ax Kalman'],
        title = 'X Acceleration (m/s^2) vs Time',
        labels={
            'value': 'X Acceleration (m/s)', 'time':'Time'
        },
    )
    sensed_dynamics_acceleration_y = px.line(
        data,
        x = 'time',
        y = ['Ay Truth', 'Ay Kalman'],
        title = 'Y Acceleration (m/s^2) vs Time',
        labels={
            'value': 'Y Acceleration (m/s^2)', 'time': 'Time'
        },
    )
    sensed_dynamics_acceleration_z = px.line(
        data,
        x = 'time',
        y = ['Az Truth', 'Az Kalman'],
        title = 'Z Acceleration (m/s^2) vs Time',
        labels={
            'value': 'Z Acceleration (m/s^2)', 'time': 'Time'
        },
    )

    sensed_dynamics_pitch = px.line(
        data,
        x = 'time',
        y = ["Pitch Truth", "Pitch Kalman", "Magnetometer Pitch"],
        title = "Pitch (degrees) vs Time",
        labels={
            'value': "Pitch (degrees)", 'time': "Time",
        }
    )

    sensed_dynamics_roll = px.line(
        data,
        x = 'time',
        y = ["Roll Truth", "Roll Kalman", "Magnetometer Roll"],
        title = "Roll (degrees) vs Time",
        labels={
            'value': "Roll (degrees)", 'time': "Time",
        }
    )

    sensed_dynamics_yaw = px.line(
        data,
        x = 'time',
        y = ["Yaw Truth", "Yaw Kalman", "Magnetometer Yaw"],
        title = "Yaw (degrees) vs Time",
        labels={
            'value': "Yaw (degrees)", 'time': "Time",
        }
    )

    sensed_dynamics_pitch_rate = px.line(
        data,
        x = 'time',
        y = ["Omega Pitch Truth", "Omega Pitch Kalman", "Gyro Pitch"],
        title = "Pitch Rate (deg/s) vs Time",
        labels={
            'value': 'Pitch Rate (deg/s)', 'time': 'Time'
        }
    )

    sensed_dynamics_yaw_rate = px.line(
        data,
        x = 'time',
        y = ["Omega Yaw Truth", "Omega Yaw Kalman", "Gyro Yaw"],
        title = "Yaw Rate (deg/s) vs Time",
        labels={
            'value': 'Yaw Rate (deg/s)', 'time': 'Time'
        }
    )

    sensed_dynamics_roll_rate = px.line(
        data,
        x = 'time',
        y = ["Omega Roll Truth", "Omega Roll Kalman", "Gyro Roll"],
        title = "Roll Rate (deg/s) vs Time",
        labels={
            'value': 'Roll Rate (deg/s)', 'time': 'Time'
        }
    )

    sensed_dynamics_pitch_acceleration = px.line(
        data,
        x = 'time',
        y = ["Alpha Pitch Truth", "Alpha Pitch Kalman"],
        title = "Pitch Acceleration (deg/s^2)",
        labels={
            'value': 'Pitch Acceleration (deg/s^2)'
        }
    )
    
    sensed_dynamics_yaw_acceleration = px.line(
        data,
        x = 'time',
        y = ["Alpha Yaw Truth", "Alpha Yaw Kalman"],
        title = "Yaw Acceleration (deg/s^2)",
        labels={
            'value': 'Yaw Acceleration (deg/s^2)', 'time':'Time'
        }
    )
    
    sensed_dynamics_roll_acceleration = px.line(
        data,
        x = 'time',
        y = ["Alpha Roll Truth", "Alpha Roll Kalman"],
        title = "Roll Acceleration (deg/s^2)",
        labels={
            'value': 'Roll Acceleration (deg/s^2)', 'time':'Time'
        }
    )
    

    app.layout = html.Div(
        children=[
            html.Div(
                className="d ",
                children=[
                    html.H1(children="SSI Fountain Hopper Sim"),
                    html.Img(
                        height=100,
                        src="https://www.stanfordssi.org/images/LogoOnly.svg",
                    ),
                ],
            ),
            html.Div([
                #ALTITUDE
                html.H2(children='Altitude'),
                dcc.Graph(id="Altitude", figure=altitude, style={'width':'80%', 'margin': '0 auto'}),

                #POSITION ERROR
                html.H2(children="Position Error"),
                html.Div([
                    dcc.Graph(id="X Position Error", figure=position_x, style={'display': 'inline-block', 'width':'32%'}),
                    dcc.Graph(id="Y Position Error", figure=position_y, style={'display': 'inline-block', 'width':'32%'}),
                    dcc.Graph(id="Z Position Error", figure=position_z, style={'display': 'inline-block', 'width':'32%'}),
                ], className="row"),

                #ROTATION ERROR
                html.H2(children="Rotation Error"),
                html.Div([
                    dcc.Graph(id="Pitch Error (degrees) vs Time", figure=pitch_error, style={'width':'32%', 'display': 'inline-block'}),
                    dcc.Graph(id="Roll Error (degrees) vs Time", figure=yaw_error, style={'width':'32%', 'display': 'inline-block'}),
                    dcc.Graph(id="Yaw Error (degrees) vs Time", figure=roll_error, style={'width':'32%', 'display': 'inline-block'}),
                ], className="row"),

                #GIMBAL ERRORS
                html.H2(children="GIMBAL ERRORS"),
                html.Div([
                    dcc.Graph(id="Gimbal Psi (degreens) vs Time", figure=gimbal_psi, style={'width':'32%', 'display': 'inline-block'}),
                    dcc.Graph(id="Gimbal Theta (degreens) vs Time", figure=gimbal_theta, style={'width':'32%', 'display': 'inline-block'}),
                    dcc.Graph(id="Throttle Percent vs. Time", figure=throttle, style={'width':'32%', 'display': 'inline-block'}),
                ], className="row"),

                #GIMBAL POSTINGS
                html.H2(children="GIMBAL POSTINGS"),
                html.Div([
                    dcc.Graph(id="PosX (m) vs Time", figure=posX, style={'width':'50%', 'display': 'inline-block'}),
                    dcc.Graph(id="PosY (m) vs Time", figure=posY, style={'width':'50%', 'display': 'inline-block'}),
                ], className="row"),

                #CONTROL INPUTS (U)
                html.H2(children="CONTROL INPUTS (U)"),
                html.Div([
                    dcc.Graph(id="Ux (m/s^2 vs Time)", figure=ux, style={'width':'32%', 'display': 'inline-block'}),
                    dcc.Graph(id="Uy (m/s^2 vs Time)", figure=uy, style={'width':'32%', 'display': 'inline-block'}),
                    dcc.Graph(id="Uz (m/s^2 vs Time)", figure=uz, style={'width':'32%', 'display': 'inline-block'}),
                ], className="row"),

                #MOI 
                html.H2(children="MOI"),
                html.Div([
                    dcc.Graph(id="IXX (kgm2) vs Time", figure=moixx, style={'width':'32%', 'display': 'inline-block'}),
                    dcc.Graph(id="IYY (kgm2) vs Time", figure=moiyy, style={'width':'32%', 'display': 'inline-block'}),
                    dcc.Graph(id="IZZ (kgm2) vs Time", figure=moizz, style={'width':'32%', 'display': 'inline-block'}),
                ], className="row"),

                #DYNAMICS
                html.H2(children="DYNAMICS"),
                #positions
                html.H4(children="Positions"),
                html.Div([
                    dcc.Graph(id="X Positon (m) vs. Time", figure=x_position, style={'width':'32%', 'display': 'inline-block'}),
                    dcc.Graph(id="Y Positon (m) vs. Time", figure=y_position, style={'width':'32%', 'display': 'inline-block'}),
                    dcc.Graph(id="Z Positon (m) vs. Time", figure=z_position, style={'width':'32%', 'display': 'inline-block'}),
                ], className="row"),
                #velocities
                html.H4(children="Velocities"),
                html.Div([
                    dcc.Graph(id="X Velocity (m) vs. Time", figure=x_velocity, style={'width':'32%', 'display': 'inline-block'}),
                    dcc.Graph(id="Y Velocity (m) vs. Time", figure=y_velocity, style={'width':'32%', 'display': 'inline-block'}),
                    dcc.Graph(id="Z Velocity (m) vs. Time", figure=z_velocity, style={'width':'32%', 'display': 'inline-block'}),
                ], className="row"),
                #accelerations
                html.H4(children="Velocities"),
                html.Div([
                    dcc.Graph(id="X Acceleration (m) vs. Time", figure=x_acceleration, style={'width':'32%', 'display': 'inline-block'}),
                    dcc.Graph(id="Y Acceleration (m) vs. Time", figure=y_acceleration, style={'width':'32%', 'display': 'inline-block'}),
                    dcc.Graph(id="Z Acceleration (m) vs. Time", figure=z_acceleration, style={'width':'32%', 'display': 'inline-block'}),
                ], className="row"),

                #ROTATIONAL DYNAMICS
                html.H2(children='ROTATIONAL DYNAMICS'),
                #degrees theta
                html.H4(children="Degrees"),
                html.Div([
                    dcc.Graph(id="Pitch (degrees) vs. Time", figure=pitch_degrees, style={'width':'32%', 'display': 'inline-block'}),
                    dcc.Graph(id="Yaw (degrees) vs. Time", figure=yaw_degrees, style={'width':'32%', 'display': 'inline-block'}),
                    dcc.Graph(id="Roll (degrees) vs. Time", figure=roll_degrees, style={'width':'32%', 'display': 'inline-block'}),
                ], className="row"),

                #omega
                html.H4(children='Omega'),
                html.Div([
                    dcc.Graph(mathjax = True, id="Omega Pitch (deg/s) vs. Time", figure=pitch_omega, style={'width':'32%', 'display': 'inline-block'}),
                    dcc.Graph(mathjax = True, id="Omega Yaw (deg/s) vs. Time", figure=yaw_omega, style={'width':'32%', 'display': 'inline-block'}),
                    dcc.Graph(id="Omega Roll (deg/s) vs. Time", figure=roll_omega, style={'width':'32%', 'display': 'inline-block'}),
                ], className="row"),

                #alpha
                html.H4(children='Alpha'),
                html.Div([
                    dcc.Graph(id="Alpha Pitch (deg/s) vs. Time", figure=pitch_alpha, style={'width':'32%', 'display': 'inline-block'}),
                    dcc.Graph(id="Alpha Yaw (deg/s) vs. Time", figure=yaw_alpha, style={'width':'32%', 'display': 'inline-block'}),
                    dcc.Graph(id="Alpha Roll (deg/s) vs. Time", figure=roll_alpha, style={'width':'32%', 'display': 'inline-block'}),
                ], className="row"),

                #WIND
                html.H2(children='Wind'),
                html.Div([
                dcc.Graph(id="Wind vs Time", figure=wind, style={'width':'80%', 'margin': '0 auto'}),
                ], className="row"),

                #LANDING POSITION
                html.H2(children='Landing Position'),
                html.Div([
                dcc.Graph(id="Landing Position", figure=landing_positon, style={'height':'700px', 'width':'700px', 'margin': '0 auto'}),
                ], className="row", style={'height':'700px'}),

                #SENSED DYNAMICS                
                html.H2(children = 'Sensed Dynamics - Movement'),
                
                #Dynamics accross positon 
                html.H4(children = 'Positions'),
                html.Div([
                dcc.Graph(id="X Position (m) vs Time", figure=sensed_dynamics_positon_x, style={'width':'32%', 'display': 'inline-block'}),
                dcc.Graph(id="Y Position (m) vs Time", figure=sensed_dynamics_positon_y, style={'width':'32%', 'display': 'inline-block'}),
                dcc.Graph(id="Z Position (m) vs Time", figure=sensed_dynamics_positon_z, style={'width':'32%', 'display': 'inline-block'}),
                ], className="row"),

                #Dynamics accross velocity
                html.H4(children='Velocities'),
                dcc.Graph(id="X Velocity (m/s) vs Time", figure=sensed_dynamics_velocity_x, style={'width':'32%', 'display': 'inline-block'}),
                dcc.Graph(id="Y Velocity (m/s) vs Time", figure=sensed_dynamics_velocity_y, style={'width':'32%', 'display': 'inline-block'}),
                dcc.Graph(id="Z Velocity (m/s) vs Time", figure=sensed_dynamics_velocity_z, style={'width':'32%', 'display': 'inline-block'}),

                #Dynamics accross acceleration
                html.H4(children='Accelerations'),
                dcc.Graph(id="X Acceleration (m/s) vs Time", figure=sensed_dynamics_acceleration_x, style={'width':'32%', 'display': 'inline-block'}),
                dcc.Graph(id="Y Acceleration (m/s) vs Time", figure=sensed_dynamics_acceleration_y, style={'width':'32%', 'display': 'inline-block'}),
                dcc.Graph(id="Z Acceleration (m/s) vs Time", figure=sensed_dynamics_acceleration_z, style={'width':'32%', 'display': 'inline-block'}),

                #ROTATIONAL DYNAMICS
                #Degrees
                html.H4(children = 'Rotation'),
                dcc.Graph(id="Pitch (degrees) vs Time", figure=sensed_dynamics_pitch, style={'width':'32%', 'display': 'inline-block'}),
                dcc.Graph(id="Roll (degrees) vs Time", figure=sensed_dynamics_yaw, style={'width':'32%', 'display': 'inline-block'}),
                dcc.Graph(id="Yaw (degrees) vs Time", figure=sensed_dynamics_roll, style={'width':'32%', 'display': 'inline-block'}),

                #Omega
                html.H4(children = 'Rotation Rate'),
                dcc.Graph(id="Pitch Rate (deg/s) vs Time", figure=sensed_dynamics_pitch_rate, style={'width':'32%', 'display': 'inline-block'}),
                dcc.Graph(id="Pitch Rate (deg/s) vs Time", figure=sensed_dynamics_yaw_rate, style={'width':'32%', 'display': 'inline-block'}),
                dcc.Graph(id="Pitch Rate (deg/s) vs Time", figure=sensed_dynamics_roll_rate, style={'width':'32%', 'display': 'inline-block'}),

                #Alpha
                #NEw line
                html.H4(children = "Rotational Acceleration"),
                dcc.Graph(id="Pitch Rate (deg/s) vs Time", figure=sensed_dynamics_pitch_acceleration, style={'width':'32%', 'display': 'inline-block'}),
                dcc.Graph(id="Pitch Rate (deg/s) vs Time", figure=sensed_dynamics_yaw_acceleration, style={'width':'32%', 'display': 'inline-block'}),
                dcc.Graph(id="Pitch Rate (deg/s) vs Time", figure=sensed_dynamics_roll_acceleration, style={'width':'32%', 'display': 'inline-block'}),

                

 
            ], className="row"),
            
        ]
    )

    app.run(debug=True)
