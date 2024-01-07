import numpy as np
from scipy.spatial.transform import Rotation
from Simulator.simulationConstants import GRAVITY as g
from Simulator.simulationConstants import RHO as rho

def natural_dyanamics(state, rocket, wind, dt):
    """ Uncontrolled dynamics of rocket """
    # Calculate Accelerations in rocket frame
    # Pull Params
    m = rocket.mass
    v = state[3:6]
    w = state[9:12]

    # Build Statedot
    statedot = np.zeros(len(state))
    statedot[0:3] = v
    statedot[6:9] = w
    
    # Rocket rotations
    pitch = state[6] # Angle from rocket from pointing up towards positive x axis
    yaw = state[7] # Angle from rocket from pointing up towards positive y axis
    roll = state[8] # Roll, ccw when looking down on rocket
    R = Rotation.from_euler('xyz', [yaw, -pitch, -roll]).as_matrix()
    R_inv = np.linalg.inv(R)

    # Wind rotation into rocket frame
    wind_rf = np.dot(R, wind + v)
    wind_force = rocket.find_wind_force(wind_rf, rho)
    wind_moment = rocket.find_wind_moment(wind_rf, rho)

    # Calculate Accelerations in rocket frame
    aX_rf = (-1 * g * R[0][2]) + (wind_force[0] / m)
    aY_rf = (-1 * g * R[1][2]) + (wind_force[1] / m)
    aZ_rf = (-1 * g * R[2][2]) + (wind_force[2] / m)
    a_rf = np.array([aX_rf, aY_rf, aZ_rf])

    # Convert Accelerations from rocket frame to global frame
    a_global = np.dot(R_inv, a_rf)

    # Calculate Alphas
    torque = np.array([wind_moment[0],
                       wind_moment[1],
                        0])
    I_dot = (rocket.I - rocket.I_prev) / dt
    alphas = np.dot(np.linalg.inv(rocket.I), torque - np.cross(w, np.dot(rocket.I, w)) - np.dot(I_dot, w))

    statedot[3:6] = a_global.tolist()
    statedot[9:12] = alphas.tolist()

    return statedot

def controlled_dynamics_2(state, rocket, dt, t, acc_x, acc_y, acc_z):
    """ Rocket dynamics with control"""
    # Pull Params
    m = rocket.mass
    lever_arm = rocket.com
    engine_length = rocket.engine.length
    v = state[3:6]
    w = state[9:12]

    # Build Statedot
    statedot = np.zeros(len(state))
    statedot[0:3] = v
    statedot[6:9] = w
    
    # Rocket rotations
    pitch = state[6] # Angle from rocket from pointing up towards positive x axis
    yaw = state[7] # Angle from rocket from pointing up towards positive y axis
    roll = state[8] # Roll, ccw when looking down on rocket
    R = Rotation.from_euler('xyz', [yaw, -pitch, -roll]).as_matrix()
    R_inv = np.linalg.inv(R)

    # Calculate Accelerations in rocket frame
    a_rf = np.array([acc_x, acc_y, acc_z])

    # Convert Accelerations from rocket frame to global frame
    a_global = np.dot(R_inv, a_rf)

    # Calculate Alphas
    torque = np.array([(acc_x * m * lever_arm),
                        (acc_y * m * lever_arm),
                        0])
    I_dot = (rocket.I - rocket.I_prev) / dt
    alphas = np.dot(np.linalg.inv(rocket.I), torque - np.cross(w, np.dot(rocket.I, w)) - np.dot(I_dot, w))

    statedot[3:6] = a_global.tolist()
    statedot[9:12] = alphas.tolist()

    return statedot



def controlled_dynamics(state, rocket, dt, t, posX, posY, T):
    """ Rocket dynamics with control"""
    # Pull Params
    m = rocket.mass
    lever_arm = rocket.com
    engine_length = rocket.engine.length
    v = state[3:6]
    w = state[9:12]

    # Convert Actuator Positions to Cyclindrical Coords
    gimbal_R = np.sqrt((posX ** 2) + (posY ** 2))
    gimbal_theta = np.arctan2(posY, posX)
    gimbal_psi = np.arctan2(gimbal_R, engine_length)

    # Build Statedot
    statedot = np.zeros(len(state))
    statedot[0:3] = v
    statedot[6:9] = w
    
    # Rocket rotations
    pitch = state[6] # Angle from rocket from pointing up towards positive x axis
    yaw = state[7] # Angle from rocket from pointing up towards positive y axis
    roll = state[8] # Roll, ccw when looking down on rocket
    R = Rotation.from_euler('xyz', [yaw, -pitch, -roll]).as_matrix()
    R_inv = np.linalg.inv(R)

    # Calculate Accelerations in rocket frame
    aX_rf = (T * np.sin(gimbal_psi) * np.cos(gimbal_theta) / m)
    aY_rf = (T * np.sin(gimbal_psi) * np.sin(gimbal_theta) / m)
    aZ_rf = (T * np.cos(gimbal_psi) / m)
    a_rf = np.array([aX_rf, aY_rf, aZ_rf])

    # Convert Accelerations from rocket frame to global frame
    a_global = np.dot(R_inv, a_rf)

    # Calculate Alphas
    torque = np.array([(T * np.sin(gimbal_psi) * np.cos(gimbal_theta) * lever_arm),
                        (T * np.sin(gimbal_psi) * np.sin(gimbal_theta) * lever_arm),
                        0])
    I_dot = (rocket.I - rocket.I_prev) / dt
    alphas = np.dot(np.linalg.inv(rocket.I), torque - np.cross(w, np.dot(rocket.I, w)) - np.dot(I_dot, w))

    statedot[3:6] = a_global.tolist()
    statedot[9:12] = alphas.tolist()

    return statedot

def full_dynamics(state, rocket, wind, dt, t):

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
    statedot[6:9] = w
    
    # Rocket rotations
    pitch = state[6] # Angle from rocket from pointing up towards positive x axis
    yaw = state[7] # Angle from rocket from pointing up towards positive y axis
    roll = state[8] # Roll, ccw when looking down on rocket
    R = Rotation.from_euler('xyz', [yaw, -pitch, -roll]).as_matrix()
    R_inv = np.linalg.inv(R)

    # Wind rotation into rocket frame
    wind_rf = np.dot(R, wind + v)
    wind_force = rocket.find_wind_force(wind_rf, rho)
    wind_moment = rocket.find_wind_moment(wind_rf, rho)

    # Calculate Accelerations in rocket frame
    aX_rf = (T * np.sin(gimbal_psi) * np.cos(gimbal_theta) / m) + (-1 * g * R[0][2]) + (wind_force[0] / m)
    aY_rf = (T * np.sin(gimbal_psi) * np.sin(gimbal_theta) / m) + (-1 * g * R[1][2]) + (wind_force[1] / m)
    aZ_rf = (T * np.cos(gimbal_psi) / m) + (-1 * g * R[2][2]) + (wind_force[2] / m)
    a_rf = np.array([aX_rf, aY_rf, aZ_rf])

    # Convert Accelerations from rocket frame to global frame
    a_global = np.dot(R_inv, a_rf)

    # Calculate Alphas
    torque = np.array([(T * np.sin(gimbal_psi) * np.cos(gimbal_theta) * lever_arm) + wind_moment[0],
                        (T * np.sin(gimbal_psi) * np.sin(gimbal_theta) * lever_arm) + wind_moment[1],
                        0])
    I_dot = (rocket.I - rocket.I_prev) / dt
    alphas = np.dot(np.linalg.inv(rocket.I), torque - np.cross(w, np.dot(rocket.I, w)) - np.dot(I_dot, w))

    statedot[3:6] = a_global.tolist()
    statedot[9:12] = alphas.tolist()

    return statedot
    