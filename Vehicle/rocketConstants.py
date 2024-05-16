ROCKET_MASS_TOTAL = 98.03
ROCKET_MASS_WITHOUT_ENGINE = 36.8
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

# Sensors
ACCELEROMETER_SIGMA = 0.001
ACCELEROMETER_MU = 0.00
ACCELEROMETER_UPDATE_FREQ = 100 #hertz

# GYRO
GYROSCOPE_SIGMA = 0.001
GYROSCOPE_MU = 0
GYROSCOPE_UPDATE_FREQ = 100

# Magetometer
MAGNETOMETER_SIGMA = 0.01
MAGNETOMETER_MU = 0.000
MAGNETOMETER_UPDATE_FREQ = 100 #hertz

# Barometer
BAROMETER_SIGMA = 0.001
BAROMETER_MU = 0.000
BAROMETER_UPDATE_FREQ = 100 #hertz

# GPS
GPS_SIGMA = 0.001 # Assuming "10mm accuracy" from spec sheet is refering to it's standard deviation
GPS_MU = 0 # Not biased so far as we can tell
GPS_UPDATE_FREQ = 1 #hertz


