import numpy as np
import os
import scipy.integrate
import Vehicle.engine
import Vehicle.rocket
import control as ct
from Vehicle.sensors import *
from Simulator.constraints import *
from scipy.spatial.transform import Rotation
from Simulator.dynamics import *
from Simulator.errorInjection import *
from Simulator.simulationConstants import GRAVITY as g
from Simulator.simulationConstants import RHO as rho
from scipy.spatial.transform import Rotation
from Vehicle.ActuatorModel import *
from termcolor import colored

class Simulation:
    """ Class Representing the Simulation and associated data"""
    def __init__(self, timefinal, simulation_timestep, starting_state, wind, planned_trajectory):                      
        
        # Create Rocket Object
        self.rocket = Vehicle.rocket.Rocket(simulation_timestep, planned_trajectory, starting_state)
        self.ideal_trajectory = planned_trajectory
        self.state = roll_injection(starting_state)
        
        # Simulation Variables
        self.ts = simulation_timestep
        self.tf = timefinal
        self.previous_time = 0
        self.current_step = 0
        self.landed = False
        self.landing_violation = None
        
        # Initialize situation
        self.wind_history = np.empty((0,len(wind)))
        self.base_wind = np.array([np.random.normal(0, wind[0]), np.random.normal(0, wind[1]), np.random.normal(0, wind[2])])
        self.current_wind = self.base_wind                     

    def propogate(self):
        """ Simple propogator

        Inputs:
            state_0 = initial state
            tf = end time of simulation
            ts = timestep

        Returns:
            solution = position data at each timestamp
            
        """
        # Pull Time Information
        tf = self.tf
        ts = self.ts

        state = self.state

        # Create t vector from 0 to tf with timestep ts
        t_span = np.linspace(0, int(tf/ts)*ts, num=int(tf/ts)+1)
        t = (0,self.tf)

        # Propogate given ODE, stop when rocket crashes as indicated by this here event function
        def event(t,y,r,it,tt):
            if t < 10 * ts: # Prevent from thinking it's crashed when sitting on ground on first 10 time steps
                return 1
            else:
                if self.landed == True:
                    return 0
                else:
                    return y[2]
        event.terminal=True
        event.direction=-1
        try:
            solution = scipy.integrate.solve_ivp(self.wrapper_state_to_stateDot, t, state, args=(self.rocket, self.ideal_trajectory, t_span), t_eval=t_span, max_step=ts/5, events=event)
            state_history = solution['y'].T
        except:
            state_history = self.rocket.state_history
            
        return state_history

    def display_end_info(self):
        """ Display ending info about the simulation into the terminal"""
        
        os.system('color')
        print()
        if self.landed:
            print(colored("Sucsessful Execution of Planned Trajectory and Landing", "green"))
        else:
            if self.landing_violation:
                print(colored("Landing Condition Violation: ", "red"))
                print(self.landing_violation)
            else:
                print(colored("Vehicle Crash Detected", "red"))
        
        print()
        print("Simulation Ended at t = ", self.previous_time, "s, at simulation step ", self.current_step)
        print()
        print("Rocket Start Mass: ", self.rocket.massHistory[0], "kg | End Mass: ", self.rocket.mass)
        print("Engine Start Mass: ", self.rocket.engine.full_mass, "kg | End Mass: ", self.rocket.engine.mass)
        print("Percent Fuel Remaining: ", (1 - (self.rocket.engine.full_mass - self.rocket.engine.mass) / (self.rocket.engine.full_mass - self.rocket.engine.drymass)) * 100, "%")
        print()

    def check_landing(self, state, t):
        """ Check if the landing was valid"""
        # Check if below threshold altitude
        if state[2] < THRESHOLD_ALTITUDE and t > THRESHOLD_TIME:
            # Check Z speed
            if abs(state[5]) < MAX_Z_SPEED:
                # Check XY speed
                if abs(state[4]) < MAX_XY_SPEED and abs(state[3]) < MAX_XY_SPEED:
                    # Check rotation
                    if abs(state[6]) < MAX_ROTATION and abs(state[7]) < MAX_ROTATION: 
                
                        if abs(state[9]) < MAX_ROTATION_SPEED and abs(state[10]) < MAX_ROTATION_SPEED:
                            self.landed = True
                            self.state = state
                        else:
                            self.landing_violation = "EXCESSIVE ROTATIONAL VELOCITY - {} rad/s or {} rad/s out of {} rad/s allowed".format(abs(state[9]), abs(state[10]), MAX_ROTATION_SPEED)
                    else:
                        self.landing_violation = "EXCESSIVE TILT - {} rad or {} rad out of {} rad allowed".format(abs(state[6]), abs(state[7]), MAX_ROTATION)
                else:
                    self.landing_violation = "EXCESSIVE LATERAL SPEED - {} m/s or {} m/s out of {} m/s allowed".format(abs(state[3]), abs(state[4]), MAX_XY_SPEED) 
            else:
                self.landing_violation = "EXCESSIVE DESCENT SPEED - {} m/s out of {} m/s allowed".format(abs(state[5]), MAX_Z_SPEED)
        elif t > 2 and state[2] < THRESHOLD_ALTITUDE:
            self.landing_violation = "OFF COURSE: t={}, alt={}".format(t, state[2])
                               
    def wrapper_state_to_stateDot(self, t, state, rocket, ideal_trajectory, t_vec):
        """ Wrapper for the dynamics, most of the work done in this step. It calls the controller and updates the rocket's state based on
        control inputs and fuel drain."""
        # Check if we are on an actual simulation timestep or if this is ode solving shenanigans
        if (t == 0) or (t >= t_vec[self.current_step] and self.previous_time < t_vec[self.current_step]):
            
            rocket.state = state

            # Determine wind at this moment in time
            self.current_wind = wind_randomness(self.base_wind, self.current_wind)
            self.wind_history = np.vstack([self.wind_history, self.current_wind])

            rocket.ffc.rocket_loop(t, self.current_step)
            rocket.update_truth_side(t, self.ts, self.current_step)
            
            if not t == t_vec[-1]:
                self.current_step += 1
            self.previous_time = t
            
            # Check for landing
            self.check_landing(rocket.state, t)
        
        self.statedot_previous = full_dynamics(rocket.state, rocket, self.current_wind, self.ts, t)
        return self.statedot_previous