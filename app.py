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


def run_dash(sim, planned_trajectory, trajectory, ts, tf):
    app = Dash(__name__)

    # assume you have a "long-form" data frame
    # see https://plotly.com/python/px-arguments/ for more options

    t_length = len(sim.rocket.state_history[:, 2])
    data = pd.DataFrame(
        {
            "time": np.linspace(0, tf, t_length),
            "Rocket State History": sim.rocket.state_history[:, 2],
            "FFC State History": sim.rocket.ffc.state_history[:, 2],
            "X Truth position_error": sim.rocket.error_history[:, 0],
            "Y Truth position_error": sim.rocket.error_history[:, 1],
            "Z Truth position_error": sim.rocket.error_history[:, 2],
            "X FFC position_error": sim.rocket.ffc.error_history[:, 0],
            "Y FFC position_error": sim.rocket.ffc.error_history[:, 1],
            "Z FFC position_error": sim.rocket.ffc.error_history[:, 2],
        }
    )

    altitude = px.line(
        data,
        x="time",
        y=["Rocket State History", "FFC State History"],
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
        y=["Z Truth position_error", "Y FFC position_error"],
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
            dcc.Graph(id="Altitude", figure=altitude),
            dcc.Graph(id="X Position Error", figure=position_x),
            dcc.Graph(id="Y Position Error", figure=position_y),
            dcc.Graph(id="Z Position Error", figure=position_z),
        ]
    )

    app.run(debug=True)
