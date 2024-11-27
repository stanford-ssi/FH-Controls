import numpy as np
import random
from scipy.spatial.transform import Rotation
from Simulator.simulationConstants import GRAVITY as g
from Simulator.simulationConstants import RHO as rho
from Simulator.simulationConstants import *
from Simulator.errorInjection import *

def full_dynamics(state, rocket, wind, dt, t):
    """ These are the dynamics used during simulation. It is the full dynamics including wind.
        
        Inputs:
        - state in gf
        - rocket object
        - wind vector in global frame (1x3)
        - timestep length
        - current time
        
        Outputs:
        - statedot vector (1x12)
    """
    # Pull Params
    throttle = rocket.engine.throttle
    T = rocket.engine.get_thrust(t, throttle)
    m = rocket.mass
    lever_arm = rocket.com
    engine_length = rocket.engine.length
    posX = rocket.engine.posx
    posY = rocket.engine.posy
    
    v = state[3:6]
    w = state[9:12]

    # Convert Actuator Positions to Cyclindrical Coords
    gimbal_R = np.sqrt((posX ** 2) + (posY ** 2))
    gimbal_theta = np.arctan2(posY, posX)
    gimbal_psi = np.arctan2(gimbal_R, engine_length)

    # Build Statedot
    statedot = np.zeros(len(state))
    statedot[0:3] = v
    
    # Rocket rotations
    pitch = state[6] # Angle from rocket from pointing up towards positive x axis
    yaw = state[7] # Angle from rocket from pointing up towards positive y axis
    roll = state[8] # Roll, ccw when looking down on rocket
    R = Rotation.from_euler('xyz', [yaw, -pitch, -roll]).as_matrix()
    R_inv = np.linalg.inv(R)

    # Wind rotation into rocket frame
    wind_rf = np.dot(R, wind + np.dot(R, v))
    wind_force = rocket.find_wind_force(wind_rf, rho)
    wind_moment = rocket.find_wind_moment(wind_rf, rho)

    # Calculate Accelerations in rocket frame
    aX_rf = (T * np.sin(gimbal_psi) * -np.cos(gimbal_theta) / m) + (-1 * g * R[0][2]) + (wind_force[0] / m)
    aY_rf = (T * np.sin(gimbal_psi) * -np.sin(gimbal_theta) / m) + (-1 * g * R[1][2]) + (wind_force[1] / m)
    aZ_rf = (T * np.cos(gimbal_psi) / m) + (-1 * g * R[2][2]) + (wind_force[2] / m)
    a_rf = np.array([aX_rf, aY_rf, aZ_rf])

    # Convert Accelerations from rocket frame to global frame
    a_global = np.dot(R_inv, a_rf)

    # Calculate Alphas
    torque = np.array([(T * np.sin(gimbal_psi) * -np.cos(gimbal_theta) * lever_arm) + wind_moment[0],
                        (T * np.sin(gimbal_psi) * -np.sin(gimbal_theta) * lever_arm) + wind_moment[1],
                        0]) + rocket.engine.I @ rocket.engine.gimbal_alpha # This accounts for torque from the tvc system
    I_dot = (rocket.I - rocket.I_prev) / dt
    alphas = np.dot(np.linalg.inv(rocket.I), torque - np.cross(w, np.dot(rocket.I, w)) - np.dot(I_dot, w))

    statedot[3:6] = a_global.tolist()
    statedot[9:12] = alphas.tolist()
    
    # Rotational Kinematics
    statedot[6:9] = get_EA_dot(state, R, R_inv)

    return statedot

def get_EA_dot(state, R, R_inv):
    w_gf = np.dot(R_inv, state[9:12])

    wx = w_gf[0]
    wy = w_gf[1]
    wz = w_gf[2]
    pitch = state[6]
    yaw = state[7]
    roll = state[8]
    
    pitchdot = wy*np.cos(roll) - wx*np.sin(roll) 
    yawdot = (wy*np.sin(roll) + wx*np.cos(roll)) / np.cos(pitch)
    rolldot = wz - (wy*np.cos(roll) - wz*np.sin(roll)) * np.tan(pitch)
    
    return [pitchdot, yawdot, rolldot]
 