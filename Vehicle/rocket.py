import numpy as np
import math
import Vehicle.engine
import Vehicle.rocketConstants
from Vehicle.components import *
from scipy.integrate import cumtrapz
from scipy.interpolate import interp1d
from Vehicle.sensors import *


class Rocket:
    """ Class Representing the Rocket and associated data"""

    def __init__(self, simulation_timestep):
        # Create Engine Object inside Rocket
        self.dt = simulation_timestep
        self.engine = Vehicle.engine.Engine(simulation_timestep)
        self.mass_noEngine = Vehicle.rocketConstants.ROCKET_MASS_WITHOUT_ENGINE
        self.mass = Vehicle.rocketConstants.ROCKET_MASS_TOTAL  # Rocket Starts Fully Fueled
        self.massHistory = np.empty(shape=(0))

        # Pull Components List
        self.components = self.build_rocket(Vehicle.rocketConstants.COMPONENTS)

        # Cg and Cp locations, measured from top of rocket
        self.com = self.calculate_com()
        self.cop = Vehicle.rocketConstants.Cp

        # Calculate Moment of Inertia Tensor
        self.I_history = []
        self.update_rocket()
        self.I_history = []
        
        # Sensors
        self.accelerometer = Accelerometer()
        self.gyroscope = Gyroscope()

    def build_rocket(self, components):
        ''' Take in list of parts from rocket constants and build rocket'''
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

    def update_rocket(self):
        ''' Run all the functions needed to update the rocket at a new timestep'''
        
        percent_fuel_remaining = 1 - (self.engine.full_mass - self.engine.mass) / (self.engine.full_mass - self.engine.drymass)
        
        # Update fuels
        for component in self.components:
            if type(component) is ChangingSolidCylinder:
                component.current_length = component.original_length * percent_fuel_remaining
            if type(component) is ChangingHollowCylinder:
                new_volume = percent_fuel_remaining * math.pi * (component.outer_radius ** 2 - component.original_inner_radius ** 2) * component.length
                component.current_inner_radius = math.sqrt(component.outer_radius ** 2 - (new_volume / (math.pi * component.length)))
        
        self.update_mass()
        self.update_I()
        
    def update_mass(self):
        """ Outputs the expected mass based on it

        Inputs:
            throttleHistory = history of throttle

        Returns:
            mass = mass of rocket at current query time

        """
        self.engine.mass -= (self.engine.thrust * self.engine.throttle / self.engine.exhaust_velocity) * self.dt
        self.mass = self.mass_noEngine + self.engine.mass
        self.massHistory = np.append(self.massHistory, self.mass)

    def update_I(self):
        """ Function that is called to update the moment of inertia of the rocket at the current timestep"""
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
        """ Outputs the expected wind force vector [fx, fy, fz]

        Inputs:
            Wind in the rocket frame
            rho density of air

        Returns:
            force - wind force vector in rf

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

