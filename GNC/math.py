import numpy as np
from copy import deepcopy, copy
from Simulator.dynamics import natural_dyanamics, controlled_dynamics, full_dynamics, full_dynamics_controller

# def compute_A(state, rocket, wind, dt):
#     """ Compute Jacobian for State dot wrt State"""
#     h = 0.001
#     jacobian = np.zeros((len(state), len(state)))
#     for i in range(len(state)):
#         state_plus = deepcopy(state).astype(float)
#         state_minus = deepcopy(state).astype(float)
#         state_plus[i] = state_plus[i] + h
#         state_minus[i] = state_minus[i] - h
#         statedot_plus = natural_dyanamics(state_plus, rocket, wind, dt)
#         statedot_minus = natural_dyanamics(state_minus, rocket, wind, dt)
#         jacobian[i] = (statedot_plus - statedot_minus) / (2 * h)
#     return jacobian.T

def compute_A(state, u, rocket, wind, dt):
    """ Compute Jacobian for State dot wrt State"""
    h = 0.001
    jacobian = np.zeros((len(state), len(state)))
    for i in range(len(state)):
        state_plus = deepcopy(state).astype(float)
        state_minus = deepcopy(state).astype(float)
        state_plus[i] = state_plus[i] + h
        state_minus[i] = state_minus[i] - h
        statedot_plus = full_dynamics_controller(state_plus, rocket, wind, dt, u[0], u[1], u[2])
        statedot_minus = full_dynamics_controller(state_minus, rocket, wind, dt, u[0], u[1], u[2])
        jacobian[i] = (statedot_plus - statedot_minus) / (2 * h)
    return jacobian.T

def compute_B(state, linearized_u, rocket, dt):
    """ Compute Jacobian for State dot wrt State"""
    h = 0.001
    jacobian = np.zeros((len(linearized_u), len(state)))
    for i in range(len(linearized_u)):
        u_plus = deepcopy(linearized_u).astype(float)
        u_minus = deepcopy(linearized_u).astype(float)
        u_plus[i] = linearized_u[i] + h
        u_minus[i] = linearized_u[i] - h
        statedot_plus = controlled_dynamics(state, rocket, dt, u_plus[0], u_plus[1], u_plus[2])
        statedot_minus = controlled_dynamics(state, rocket, dt, u_minus[0], u_minus[1], u_minus[2])
        jacobian[i] = (statedot_plus - statedot_minus) / (2 * h)
    return jacobian.T




"""
    def state_to_stateDot(self, t, state, rocket):

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
        wind_rf = np.dot(R, self.wind + v)
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
        I_dot = (rocket.I - rocket.get_I_previous()) / self.timestep
        alphas = np.dot(rocket.I_inv, torque - np.cross(w, np.dot(rocket.I, w)) - np.dot(I_dot, w))

        statedot[3:6] = a_global.tolist()
        statedot[9:12] = alphas.tolist()

        return statedot
    
"""