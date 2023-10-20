import numpy as np
import scipy.integrate
import Simulator.ode

def propogate(state_0, tf, ts):
    """ Simple propogator

    Inputs:
        state_0 = initial state
        tf = end time of simulation
        ts = timestep

    Returns:
        solution = position data at each timestamp
        
    """

    # Create t vector from 0 to tf with timestep ts
    t = np.linspace(0, int(tf/ts)*ts, num=int(tf/ts))


    # Propogate given ODE
    solution = scipy.integrate.odeint(Simulator.ode.state_to_stateDot, state_0, t)
    return solution