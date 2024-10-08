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
                        0])
    I_dot = (rocket.I - rocket.I_prev) / dt
    alphas = np.dot(np.linalg.inv(rocket.I), torque - np.cross(w, np.dot(rocket.I, w)) - np.dot(I_dot, w))

    statedot[3:6] = a_global.tolist()
    statedot[9:12] = alphas.tolist()
    
    # Rotational Kinematics
    statedot[6:9] = get_EA_dot(state)

    return statedot

def get_EA_dot(state):
    
    R = Rotation.from_euler('xyz', [state[7], -state[6], -state[8]]).as_matrix()
    R_inv = np.linalg.inv(R)
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
 
def accelerations_2_actuator_positions(U_gf, rocket, t):
    """ Convert control input to engine position and throttle
    
    Inputs:
    - Control input in global frame (1x3)
    - rocket object
    - current time
    
    Output:
    - pos_x, the x position of the engine
    - pos_y, the y position of the engine
    - throttle, the throttle percent of the engine
    """
    # Rotate into rocket frame
    U = np.dot(rocket.R, U_gf) 
    gimbal_theta = np.arctan2(-U[1], -U[0])
    gimbal_psi = np.arctan2(np.sqrt((U[1] ** 2) + (U[0] ** 2)), U[2])
    T = rocket.mass * np.sqrt((U[0] ** 2) + (U[1] ** 2) + (U[2] ** 2))
    gimbal_r = np.tan(gimbal_psi) * rocket.engine.length
    if (gimbal_theta < np.pi / 2) and (gimbal_theta > -np.pi / 2):
        pos_x_commanded = np.sqrt((gimbal_r ** 2) / (1 + (np.tan(gimbal_theta) ** 2)))
    else:
        pos_x_commanded = -1 * np.sqrt((gimbal_r ** 2) / (1 + (np.tan(gimbal_theta) ** 2)))
    pos_y_commanded = pos_x_commanded * np.tan(gimbal_theta)
    throttle_commanded = rocket.engine.get_throttle(t, T)
   
    # Send signal to actuator
    pos_x = rocket.actuator_X.get_output(pos_x_commanded, t)
    pos_y = rocket.actuator_Y.get_output(pos_y_commanded, t)
    
    return pos_x, pos_y, throttle_commanded