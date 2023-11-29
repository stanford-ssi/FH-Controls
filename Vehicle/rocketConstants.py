import numpy as np

#REMEMBER TO ADD UNITS
ROCKET_MASS = 75

ROCKET_MAX_TIP = 0.015

#SHAPE: Boxes forming rocket, from top to bottom, with size. Format: (base_size_x, base_size_y, height_size)
ROCKET_SHAPE = np.array([
    (0.25, 0.25, 0.25),
    (0.25, 0.25, 2),
    (0.75, 0.75, 0.75)
])
