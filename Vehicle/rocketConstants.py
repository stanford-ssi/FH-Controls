import numpy as np

#REMEMBER TO ADD UNITS
ROCKET_MASS = 75
ROCKET_DIAMETER = 0.25
ROCKET_MAX_TIP = 0.015

# Aerodynamics, pull these values from open rocket
Cna = 2 
Cp = 1 # m from top of rocket
Cd = 0.26
AREF = 0.0095 #square m

### Compoments Section ####
COMPONENTS = [
    # Fuel Grain
    {
        'name': 'fuel_grain',
        'type': 'ChangingHollowCylinder',
        'start_mass': 40,
        'start_inner_radius': 0.01,
        'outer_radius': 0.25,
        'length': 1,
        'bottom_z': 0        
    },
    
    # Ox Tank
    {
        'name': 'ox_tank',
        'type': 'ChangingSolidCylinder',
        'start_mass': 40,
        'radius': 0.25,
        'start_length': 0.25,
        'bottom_z': 1.25
    },
    
    # N2 Tank
    {
        'name': 'n2_tank',
        'type': 'ChangingSolidCylinder',
        'start_mass': 20,
        'radius': 0.25,
        'start_length': 0.25,
        'bottom_z': 1.75
    },
    
    # Avionics
    {
        'name': 'avionics',
        'type': 'PointMass',
        'mass': 22,
        'bottom_z': 2.25
    }
]