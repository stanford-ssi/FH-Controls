ROCKET_DIAMETER = 0.25

# Aerodynamics, pull these values from open rocket
Cna = 1.08
Cp = -0.75 # m from top of rocket
Cd = 0.18
AREF = 0.0095 #square m

### Compoments Section - Build major rocket components here. z = 0 is the bottom of the rocket####
COMPONENTS = [ 
    # Fuel Grain
    {
        'name': 'fuel_grain',
        'type': 'ChangingHollowCylinder',
        'start_mass': 2.73,
        'start_inner_radius': 0.03167, # m, Updated
        'outer_radius': 0.06191, # m, Updated
        'length': 0.2831,  # Updated
        'bottom_z': 0, 
        'static': False    
    },
    
    # Fuel Casing/Combustion Chamber
    {
        'name': 'combustion_chamber',
        'type': 'HollowCylinder',
        'mass': 11.366, # kg, Updated
        'inner_radius': 0.0635, # m, Updated
        'outer_radius': 0.06858, # m, Updated
        'length': 0.6096, # meters, Updated
        'bottom_z': 0,
        'static': True       
    },
    
    # Avionics / random shit
    {
        'name': 'avionics',
        'type': 'PointMass',
        'mass': 50, # Adding extra mass for throttle help
        'bottom_z': 0.6096,
        'static': True
    },
    
    # Ox
    {
        'name': 'ox',  
        'type': 'ChangingSolidCylinder',
        'start_mass': 19.85,
        'radius': 0.25,  # Same as inner radius of Oxygen Tank
        'start_length': 0.616712,
        'bottom_z': 0.6096,  #m, Updated
        'static': False
    },
    
    # Ox Tank
    {
        'name': 'ox_tank',
        'type': 'HollowCylinder',
        'mass': 14.336, # kg, Updated
        'inner_radius': 0.09525, # m,Updated
        'outer_radius': 0.1016, # m, Updated
        'length': 1.0414, # m, Updated
        'bottom_z': 0.6096,  #m Updated minimum, because there will be feed-line components in between.
        'static': True
    },
    
        # Nitrogen
    {
        'name': 'n2',
        'type': 'ChangingSolidCylinder',
        'start_mass': 2.5,
        'radius': 0.09,
        'start_length': 0.65,
        'bottom_z': 1.25,
        'static': False
    },
    
        # Nitrogen Tank
    {
        'name': 'n2_tank',
        'type': 'HollowCylinder',
        'mass': 14.3,
        'inner_radius': 0.09,
        'outer_radius': 0.10,
        'length': 0.65,
        'bottom_z': 1.25, # Above Ox + Combustion Chamber     
        'static': True
    },
    
    # Avionics / random shit
    {
        'name': 'avionics',
        'type': 'PointMass',
        'mass': 20, # Adding extra mass for throttle help
        'bottom_z': 2.25,
        'static': True
    }
]

# Sensors
ACCELEROMETER_SIGMA = 0.1
ACCELEROMETER_MU = 0.0
ACCELEROMETER_UPDATE_FREQ = 100 #hertz

# GYRO
GYROSCOPE_SIGMA = 0.001
GYROSCOPE_MU = 0
GYROSCOPE_UPDATE_FREQ = 100

# Magetometer
MAGNETOMETER_SIGMA = 0.001
MAGNETOMETER_MU = 0.000
MAGNETOMETER_UPDATE_FREQ = 100 #hertz

# Barometer
BAROMETER_SIGMA = 1
BAROMETER_MU = 0.000
BAROMETER_UPDATE_FREQ = 100 #hertz

# GPS
GPS_SIGMA = 0.1 
GPS_MU = 0 
GPS_UPDATE_FREQ = 5 #hertz


