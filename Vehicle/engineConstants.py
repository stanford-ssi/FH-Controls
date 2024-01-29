ENGINE_DRYMASS = 55.45 #kg
ENGINE_FULL_MASS = 78.03 #kg
ENGINE_MASS_DELTA = 22.58
EXHAUST_VELOCITY = 1603 #m/s

ENGINE_BURN_DURATION = 15 #s
DT_THRUST_CURVE = 0.2 #s
THRUST_CURVE_VALUE = (ENGINE_FULL_MASS - ENGINE_DRYMASS) * EXHAUST_VELOCITY / ENGINE_BURN_DURATION
LENGTH = 0.5 #m
