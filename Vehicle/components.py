
class HollowCylinder():
    # Class for the creation of an unchanging hollow cylinder object on the rocket.
    # Component List Structure: [mass, inner radius, outer_radius, length, bottom_z]

    def __init__(self, name, mass, inner_radius, outer_radius, length, bottom_z):
        self.name = name
        self.mass = mass
        self.inner_radius = inner_radius
        self.outer_radius = outer_radius
        self.length = length
        self.bottom_z = bottom_z

    def moment_of_inertia_xy(self):
        '''
        Determines moment of Inertia of a Hollow Cylinder about an axis in the X-Y plane going through its center of mass.
        Format: [mass, inner radius, outer_radius, length, bottom_z]
        '''
        return (self.mass / 12) * ((3 * ((self.outer_radius ** 2) + (self.inner_radius ** 2))) + self.length ** 2)

    def moment_of_inertia_z(self):
        '''
        Determines moment of Inertia of a Hollow Cylinder about Z Axis.
        Format: [mass, inner radius, outer_radius, length, bottom_z]
        '''
        return 0.5 * self.mass * (self.inner_radius ** 2 + self.outer_radius ** 2)

    def center_of_mass(self):
        '''
        Determines the center of mass of a hollow cylinder.
        Return value format: [z_coord, mass]
        '''
        z_coord = self.bottom_z + self.length / 2
        return [z_coord, self.mass]


class SolidCylinder():
    # Class for the creation of an unchanging solid cylinder object on the rocket.
    # Component List Structure: [mass, radius, length, bottom_z]
    def __init__(self, name, mass, radius, length, bottom_z):
        self.name = name
        self.mass = mass
        self.radius = radius
        self.length = length
        self.bottom_z = bottom_z

    def moment_of_inertia_xy(self):
        '''
        Determines moment of Inertia of a Hollow Cylinder about an axis in the X-Y plane going through its center of mass.
        Format: [mass, radius, length, bottom_z]
        '''
        moi_xy = (0.25 * self.mass * (self.radius ** 2)) + (self.mass * (self.length ** 2) / 12)
        return moi_xy

    def moment_of_inertia_z(self):
        '''
        Determines moment of Inertia of a Solid Cylinder/Disk about Z Axis.-
        Format solid_vars: [mass, radius, length, bottom_z]'''

        return 0.5 * self.mass * (self.radius ** 2)

    def center_of_mass(self):
        '''
        Determines the center of mass of a hollow cylinder.
        Return value format: [z_coord, mass]
        '''
        z_coord = self.bottom_z + self.length / 2
        return [z_coord, self.mass]


class ChangingHollowCylinder():
    # Class for the creation of a changing hollow cylinder object on the rocket.
    # Component List Structure: [mass, current_inner_radius, outer_radius, length, bottom_z, original_inner_radius]
    def __init__(self, name, mass, current_inner_radius, outer_radius, length, bottom_z, original_inner_radius):
        self.name = name
        self.mass = mass
        self.current_inner_radius = current_inner_radius
        self.outer_radius = outer_radius
        self.length = length
        self.bottom_z = bottom_z
        self.original_inner_radius = original_inner_radius

    def moment_of_inertia_xy(self):
        '''
        Determines moment of Inertia of a Hollow Cylinder about an axis in the X-Y plane going through its center of mass.
        Format: [mass, inner radius, outer_radius, length, bottom_z]
        '''
        return (self.mass / 12) * ((3 * ((self.outer_radius ** 2) + (self.current_inner_radius ** 2))) + self.length ** 2)

    def moment_of_inertia_z(self):
        '''
        Determines moment of Inertia of a Hollow Cylinder about Z Axis.
        Format: [mass, inner radius, outer_radius, length, bottom_z]
        '''
        return 0.5 * self.mass * (self.current_inner_radius ** 2 + self.outer_radius ** 2)

    def center_of_mass(self):
        '''
        Determines the center of mass of a hollow cylinder.
        Return value format: [z_coord, mass]
        '''
        z_coord = self.bottom_z + self.length / 2
        return [z_coord, self.mass]


class ChangingSolidCylinder():
    # Class for the creation of a changing solid cylinder object on the rocket.
    # Component List Structure: [mass, radius, current length, bottom_z, original_length]
    def __init__(self, name, mass, radius, current_length, bottom_z, original_length):
        self.name = name
        self.mass = mass
        self.radius = radius
        self.current_length = current_length
        self.bottom_z = bottom_z
        self.original_length = original_length

    def moment_of_inertia_xy(self):
        '''
        Determines moment of Inertia of a Hollow Cylinder about an axis in the X-Y plane going through its center of mass.
        Format: [mass, radius, length, bottom_z]
        '''
        moi_xy = (0.25 * self.mass * (self.radius ** 2)) + (self.mass * (self.current_length ** 2) / 12)
        return moi_xy

    def moment_of_inertia_z(self):
        '''
        Determines moment of Inertia of a Solid Cylinder/Disk about Z Axis.-
        Format solid_vars: [mass, radius, length, bottom_z]'''
        return 0.5 * self.mass * (self.radius ** 2)

    def center_of_mass(self):
        '''
        Determines the center of mass of a hollow cylinder.
        Return value format: [z_coord, mass]
        '''
        z_coord = self.bottom_z + self.current_length / 2
        return [z_coord, self.mass]


class PointMass():
    # Class for the creation of an unchanging point mass object on the rocket.
    # Component List Structure: [mass, bottom_z]
    def __init__(self, name, mass, bottom_z):
        self.name = name
        self.mass = mass
        self.bottom_z = bottom_z

    def moment_of_inertia_xy(self):
        '''
        Returns component list for point mass.  Returns 0 because the Parallel Axis Theorem will compute the XY MOI in the total_moi_xy() function.
        '''
        return 0

    def moment_of_inertia_z(self):
        '''
        Assumes point mass is along z-axis.
        Returns Moment of Inertia of 0
        '''
        return 0

    def center_of_mass(self):
        '''
        Return Center of Mass of Point Mass
        Return value format: [z_coord, mass]
        '''
        return [self.bottom_z, self.mass]
