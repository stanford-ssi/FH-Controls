ROCKET_MASS_TOTAL = 98.03
ROCKET_MASS_WITHOUT_ENGINE = 36.8
ROCKET_DIAMETER = 0.25

# Aerodynamics, pull these values from open rocket
Cna = 1.08
Cp = -0.75 # m from top of rocket
Cd = 0.18
AREF = 0.0095 #square m

### Compoments Section ####
COMPONENTS = [ 
    # Fuel Grain
    {
        'name': 'fuel_grain',
        'type': 'ChangingHollowCylinder',
        'start_mass': 2.73,
        'start_inner_radius': 0.0359,
        'outer_radius': 0.0635,
        'length': 0.2728,
        'bottom_z': 0        
    },
    
    # Fuel Casing/Combustion Chamber
    {
        'name': 'combustion_chamber',
        'type': 'HollowCylinder',
        'mass': 30,
        'inner_radius': 0.0635,
        'outer_radius': 0.06858,
        'length': 0.316738,
        'bottom_z': 0        
    },
    
    # Ox
    {
        'name': 'ox_tank',
        'type': 'ChangingSolidCylinder',
        'start_mass': 19.85,
        'radius': 0.25,
        'start_length': 0.616712,
        'bottom_z': 0.55
    },
    
    # Ox Tank
    {
        'name': 'combustion_chamber',
        'type': 'HollowCylinder',
        'mass': 8.65,
        'inner_radius': 0.09525,
        'outer_radius': 0.1016,
        'length': 0.616712,
        'bottom_z': 0.55      
    },
    
        # Nitrogen
    {
        'name': 'n2_tank',
        'type': 'ChangingSolidCylinder',
        'start_mass': 2.5,
        'radius': 0.09,
        'start_length': 0.65,
        'bottom_z': 1.25
    },
    
        # Nitrogen Tank
    {
        'name': 'combustion_chamber',
        'type': 'HollowCylinder',
        'mass': 14.3,
        'inner_radius': 0.09,
        'outer_radius': 0.10,
        'length': 0.65,
        'bottom_z': 1.25      
    },
    
    # Avionics / random shit
    {
        'name': 'avionics',
        'type': 'PointMass',
        'mass': 20,
        'bottom_z': 2.25
    }
]