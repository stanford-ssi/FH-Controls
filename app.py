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
            "X FFC": ffc_dynamics[0],
            "Y FFC": ffc_dynamics[1],
            "Z FFC": ffc_dynamics[2],
            "Vx FFC": ffc_dynamics[3],
            "Vy FFC": ffc_dynamics[4],
            "Vz FFC": ffc_dynamics[5],
            "Ax FFC": ffc_dynamics[6],
            "Ay FFC": ffc_dynamics[7],
            "Az FFC": ffc_dynamics[8],
        }
    )

    altitude = px.line(
        data,
        x="time",
        y=["Altitude History", "Altitude FFC History"],
        title="Altitude",
    )
    position_x = px.line(
        data,
        x="time",
        y=["X Truth position_error", "X FFC position_error"],
        title="X Position Error",
    )
    position_y = px.line(
        data,
        x="time",
        y=["Y Truth position_error", "Y FFC position_error"],
        title="Y Position Error",
    )
    position_z = px.line(
        data,
        x="time",
        y=["Z Truth position_error", "Z FFC position_error"],
        title="Z Position Error",
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
                dcc.Graph(id="Altitude", figure=altitude),
                html.Div([
                    dcc.Graph(id="X Position Error", figure=position_x, style={'display': 'inline-block'}),
                    dcc.Graph(id="Y Position Error", figure=position_y, style={'display': 'inline-block'}),
                    dcc.Graph(id="Z Position Error", figure=position_z, style={'display': 'inline-block'}),
                ], className="row"),
            ], className="row"),
            # 
        ]
    )

    app.run(debug=True)
