import numpy as np
import math
import Vehicle.engine
import Vehicle.rocketConstants
from Vehicle.components import *
from scipy.interpolate import interp1d
from Vehicle.sensors import *
from Simulator.simulationConstants import *
from Vehicle.ActuatorModel import *
from scipy.spatial.transform import Rotation
from Vehicle.computer import *


class Rocket:
    """ Class Representing the Rocket and associated data"""

    def __init__(self, simulation_timestep, planned_trajectory, starting_state):
        # Create Engine Object inside Rocket
        self.dt = simulation_timestep
        self.engine = Vehicle.engine.Engine(simulation_timestep)
        self.mass_noEngine = Vehicle.rocketConstants.ROCKET_MASS_WITHOUT_ENGINE
        self.mass = Vehicle.rocketConstants.ROCKET_MASS_TOTAL  # Rocket Starts Fully Fueled
        self.massHistory = np.empty(shape=(0))

        # Pull Components List and Build rocket
        self.components = self.build_rocket(Vehicle.rocketConstants.COMPONENTS)

        # Find Cg and Cp locations, measured from top of rocket
        self.com = self.calculate_com()
        self.cop = Vehicle.rocketConstants.Cp

        # Calculate Moment of Inertia Tensor
        self.I_history = []
        self.update_fuels()
        self.I_history = []
        
        # Initialize the rocket's rotation matrix
        self.R_history = np.array([Rotation.from_euler('xyz', [starting_state[7], -starting_state[6], -starting_state[8]]).as_matrix()])
        self.R = np.eye(3)
        
        # Truth States and Histories
        self.state = roll_injection(starting_state)
        self.state_history = np.empty((0,len(self.state)))
        self.state_previous = copy.copy(self.state)
        self.statedot_previous = np.zeros(len(starting_state))
        self.error_history = np.empty((0,len(starting_state)))
        self.ideal_trajectory = planned_trajectory
        
        # Create Sensors
        self.accelerometer = Accelerometer(INITIAL_STATE)
        self.gyroscope = Gyroscope(INITIAL_STATE)
        self.gps = GPS(INITIAL_STATE)
        self.magnetometer = Magnetometer(INITIAL_STATE)
        self.barometer = Barometer(INITIAL_STATE)
        
        # Create Linear Actuators
        self.actuator_X = LinearActuator(FINAL_TIME, TIMESTEP)
        self.actuator_Y = LinearActuator(FINAL_TIME, TIMESTEP)
        
        # Create Flight Computer
        self.ffc = FlightComputer(starting_state, planned_trajectory, self, self.dt)
        
    def build_rocket(self, components):
        ''' Takes in list of parts from rocket constants and build rocket'''
        new_components = []
        mass_check = 0
        for component in components:
            if component['type'] == 'HollowCylinder':
                new_component = HollowCylinder(
                    component['mass'], component['inner_radius'], component['outer_radius'], component['length'], component['bottom_z'])
                mass_check += component['mass']
            if component['type'] == 'SolidCylinder':
                new_component = SolidCylinder(
                    component['mass'], component['radius'], component['length'], component['bottom_z'])
                mass_check += component['mass']
            if component['type'] == 'ChangingHollowCylinder':
                new_component = ChangingHollowCylinder(component['start_mass'], component['start_inner_radius'],
                                                       component['outer_radius'], component['length'], component['bottom_z'], component['start_inner_radius'])
                mass_check += component['start_mass']
            if component['type'] == 'ChangingSolidCylinder':
                new_component = ChangingSolidCylinder(
                    component['start_mass'], component['radius'], component['start_length'], component['bottom_z'], component['start_length'])
                mass_check += component['start_mass']
            if component['type'] == 'PointMass':
                new_component = PointMass(
                    component['mass'], component['bottom_z'])
                mass_check += component['mass']
            new_components.append(new_component)
        if not round(mass_check, 2) == round(self.mass, 2):
            print('Rocket Mass: %d' % self.mass)
            print('Sum of Components Mass: %d' % mass_check)
            raise ValueError(
                'Rocket Component Masses do not sum to total rocket mass! Check rocketConstants File!!')
        return new_components

    def calculate_com(self):
        """
        Determines the center of mass of the system (entire rocket)
        Iterates through [Z-Coordinate of center of mass, mass] for each component of the rocket.
        Returns the Z-Coordinate of the center of mass of the rocket, given Z = 0 is the bottom of the rocket.
        """
        total_mass_times_distance = 0
        total_mass = 0
        for component in self.components:
            # 2-element list, [Z-Coordinate of center of mass, mass]
            individual_com = component.center_of_mass()
            # numerator
            total_mass_times_distance += individual_com[0] * individual_com[1]
            # add mass to total mass, denominator
            total_mass += individual_com[1]
        rocket_center_of_mass = total_mass_times_distance / \
            total_mass  # calculate overall center-of-mass
        return rocket_center_of_mass

    def update_truth_side(self, t, ts, current_step):
        ''' Run all the functions needed to update the rocket at a new timestep'''
        
        self.state_history = np.vstack([self.state_history, self.state])   
        
        # Log Rocket Rotation                  
        self.R = Rotation.from_euler('xyz', [self.state[7], -self.state[6], -self.state[8]]).as_matrix()
        self.R_history = np.vstack((self.R_history, np.expand_dims(self.R.T, axis=0)))
        
        # Calculate Errors
        state_error = self.state - self.ideal_trajectory[current_step]
        self.error_history = np.vstack([self.error_history, state_error])            
        
        # Send signal to actuator
        self.engine.posx = self.actuator_X.send_signal(self.ffc.posx, t)
        self.engine.posx = self.actuator_Y.send_signal(self.ffc.posy, t)
        self.engine.throttle = self.ffc.throttle
        
        # Inject Error to actuator positions
        self.engine.posx, self.engine.posy, self.engine.throttle = actuator_error_injection(self.engine.posx, self.engine.posy, self.engine.throttle)

        # Perform actuator constraint checks
        if not t == 0:
            self.engine.throttle = throttle_checks(self.engine.throttle, self.engine.throttle_history[-1], ts)
            self.engine.posx = pos_checks(self.engine.posx, self.engine.posx_history[-1], ts)
            self.engine.posy = pos_checks(self.engine.posy, self.engine.posy_history[-1], ts)
        
        self.update_fuels()
        self.engine.save_throttle(self.engine.throttle)
        self.engine.save_posX(self.engine.posx)
        self.engine.save_posY(self.engine.posy)
        self.engine.save_thrust(self.engine.get_thrust(t, self.engine.throttle))
    
    def update_fuels(self):
        """ Remove the fuel used in the timestep from the tanks """
        # Calulate the amount of fuel remaining
        percent_fuel_remaining = 1 - (self.engine.full_mass - self.engine.mass) / (self.engine.full_mass - self.engine.drymass)
        
        # Update fuels
        for component in self.components:
            if type(component) is ChangingSolidCylinder:
                component.current_length = component.original_length * percent_fuel_remaining
            if type(component) is ChangingHollowCylinder:
                new_volume = percent_fuel_remaining * math.pi * (component.outer_radius ** 2 - component.original_inner_radius ** 2) * component.length
                component.current_inner_radius = math.sqrt(component.outer_radius ** 2 - (new_volume / (math.pi * component.length)))
        
        # Save necessary information
        self.update_mass()
        self.update_I()    
    
    def update_mass(self):
        """ Update the rocket mass
        """
        self.engine.mass -= (self.engine.thrust * self.engine.throttle / self.engine.exhaust_velocity) * self.dt
        self.mass = self.mass_noEngine + self.engine.mass
        self.massHistory = np.append(self.massHistory, self.mass)

    def update_I(self):
        """ Update the moment of inertia of the rocket at the current timestep"""
        overall_com = self.calculate_com()
        
        first_pass = False
        try:
            old_I = self.I
        except:
            first_pass = True
            
        # MOI in xy
        tot_moi_xy = 0
        for component in self.components:
            difference = abs(overall_com - component.center_of_mass()[0])
            tot_moi_xy += component.moment_of_inertia_xy() + (component.center_of_mass()[1] * (difference ** 2))

        # MOI in z
        tot_moi_z = 0
        for component in self.components:
            tot_moi_z += component.moment_of_inertia_z()
        
        # Update I and I_prev
        self.I = np.array([[tot_moi_xy, 0, 0],
                            [0, tot_moi_xy, 0],
                            [0, 0, tot_moi_z]])
        if first_pass:
            self.I_prev = self.I
        else:
            self.I_prev = old_I
        self.I_history.append(self.I)      
        
    def find_wind_force(self, wind, rho):
        """ Outputs the expected wind force vector [fx, fy, fz] based on the current state

        Inputs:
            Wind in the rocket frame
            rho density of air

        Returns:
            force - wind force vector in rocket frame

        """
        alpha = np.arccos(np.dot(wind, [0, 0, 1]) / np.linalg.norm(wind))
        Cn = Vehicle.rocketConstants.Cna * alpha
        Aref = Vehicle.rocketConstants.AREF
        Fn = 0.5 * Cn * rho * (np.linalg.norm(wind) ** 2) * \
            Aref  # Normal Force on Rocket
        Fx = Fn * wind[0] / ((np.sqrt((wind[0] ** 2) + (wind[1] ** 2))))
        Fy = Fn * wind[1] / ((np.sqrt((wind[0] ** 2) + (wind[1] ** 2))))
        Fz = 0.5 * Vehicle.rocketConstants.Cd * rho * \
            (wind[2] ** 2) * np.pi * \
            ((Vehicle.rocketConstants.ROCKET_DIAMETER / 2) ** 2)
        Force = [Fx, Fy, Fz]
        return Force

    def find_wind_moment(self, wind, rho):
        """ Outputs the expected wind moment [Mx, My, Mz]. Assumes symmetric rocket so 0 moment in roll. Can update later.

        Inputs:
            Wind in the rocket frame
            rho density of air

        Returns:
            force - wind moment vector in rf

        """

        force = self.find_wind_force(wind, rho)[0:2]
        lever_arm = self.com - self.cop
        Mx = force[0] * lever_arm
        My = force[1] * lever_arm
        moment = [Mx, My, 0]
        return moment

