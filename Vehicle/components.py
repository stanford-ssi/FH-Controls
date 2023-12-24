from abc import abstractmethod, ABC
from enum import Enum, auto
from typing import Dict



class ComponentType(Enum, ABC):

    components = {}

    @abstractmethod
    def moment_of_inertia_xy(self):
        pass

    @abstractmethod
    def moment_of_inertia_z(self):
        pass

    @abstractmethod    
    def center_of_mass(self):
        pass

    class HollowCylinder(Enum):
        #Component List Structure: [mass, inner radius, outer_radius, length, bottom_z]
        def moment_of_inertia_xy(self):
            '''
            Determines moment of Inertia of a Hollow Cylinder about an axis in the X-Y plane going through its center of mass.
            Format: [mass, inner radius, outer_radius, length, bottom_z]
            '''
            hollow_vars = ComponentType.components[ComponentType.HollowCylinder]
            mass = hollow_vars[0]
            inner_radius = hollow_vars[1]
            outer_radius = hollow_vars[2]
            length = hollow_vars[3]
            moi_xy = (mass / 12) * ((3 * ((outer_radius ** 2) + (inner_radius ** 2))) + length ** 2)

        def moment_of_inertia_z(self):
            '''
            Determines moment of Inertia of a Hollow Cylinder about Z Axis.
            Format: [mass, inner radius, outer_radius, length, bottom_z]
            '''
            hollow_vars = ComponentType.components[ComponentType.HollowCylinder]
            mass = hollow_vars[0]
            inner_radius = hollow_vars[1]
            outer_radius = hollow_vars[2]
            return 0.5 * mass * (inner_radius ** 2 + outer_radius ** 2)
        
        def center_of_mass(self):
            '''
            Determines the center of mass of a hollow cylinder.
            Return value format: [z_coord, mass]
            '''
            hollow_vars = ComponentType.components[ComponentType.HollowCylinder]
            mass = hollow_vars[0]
            bottom_z = hollow_vars[4]
            length = hollow_vars[3]
            z_coord = bottom_z + length / 2
            return [z_coord, mass]

    class SolidCylinder(Enum):
        #Component List Structure: [mass, radius, length, bottom_z]
        def moment_of_inertia_xy(self):
            '''
            Determines moment of Inertia of a Hollow Cylinder about an axis in the X-Y plane going through its center of mass.
            Format: [mass, radius, length, bottom_z]
            '''
            solid_vars = ComponentType.components[ComponentType.SolidCylinder]
            mass = solid_vars[0]
            radius = solid_vars[1]
            length = solid_vars[2]
            moi_xy = (0.25 * mass * (radius ** 2)) + (mass * (length ** 2) / 12)
            return moi_xy

        def moment_of_inertia_z(self):
            '''
            Determines moment of Inertia of a Solid Cylinder/Disk about Z Axis.-
            Format solid_vars: [mass, radius, length, bottom_z]
            '''
            solid_vars = ComponentType.components[ComponentType.SolidCylinder]
            mass = solid_vars[0]
            radius = solid_vars[1]
            return 0.5 * mass * (radius ** 2)
        
        def center_of_mass(self):
            '''
            Determines the center of mass of a hollow cylinder.
            Return value format: [z_coord, mass]
            '''
            hollow_vars = ComponentType.components[ComponentType.HollowCylinder]
            mass = hollow_vars[0]
            bottom_z = hollow_vars[3]
            length = hollow_vars[2]
            z_coord = bottom_z + length / 2
            return [z_coord, mass]
        
    class ChangingHollowCylinder(Enum):
        # Component List Structure: [mass, current_inner_radius, outer_radius, length, bottom_z, original_inner_radius]
        def moment_of_inertia_xy(self):
            '''
            Determines moment of Inertia of a Hollow Cylinder about an axis in the X-Y plane going through its center of mass.
            Format: [mass, inner radius, outer_radius, length, bottom_z]
            '''
            hollow_vars = ComponentType.components[ComponentType.HollowCylinder]
            mass = hollow_vars[0]
            inner_radius = hollow_vars[1]
            outer_radius = hollow_vars[2]
            length = hollow_vars[3]
            moi_xy = (mass / 12) * ((3 * ((outer_radius ** 2) + (inner_radius ** 2))) + length ** 2)

        def moment_of_inertia_z(self):
            '''
            Determines moment of Inertia of a Hollow Cylinder about Z Axis.
            Format: [mass, inner radius, outer_radius, length, bottom_z]
            '''
            hollow_vars = ComponentType.components[ComponentType.ChangingHollowCylinder]
            mass = hollow_vars[0]
            inner_radius = hollow_vars[1]
            outer_radius = hollow_vars[2]
            return 0.5 * mass * (inner_radius ** 2 + outer_radius ** 2)
        
        def center_of_mass(self):
            '''
            Determines the center of mass of a hollow cylinder.
            Return value format: [z_coord, mass]
            '''
            hollow_vars = ComponentType.components[ComponentType.HollowCylinder]
            mass = hollow_vars[0]
            bottom_z = hollow_vars[4]
            length = hollow_vars[3]
            z_coord = bottom_z + length / 2
            return [z_coord, mass]

    class ChangingSolidCylinder(Enum):
        # Component List Structure: [mass, radius, current length, bottom_z, original_length]
        def moment_of_inertia_xy(self):
            '''
            Determines moment of Inertia of a Hollow Cylinder about an axis in the X-Y plane going through its center of mass.
            Format: [mass, radius, length, bottom_z]
            '''
            solid_vars = ComponentType.components[ComponentType.ChangingSolidCylinder]
            mass = solid_vars[0]
            radius = solid_vars[1]
            length = solid_vars[2]
            moi_xy = (0.25 * mass * (radius ** 2)) + (mass * (length ** 2) / 12)
            return moi_xy

        def moment_of_inertia_z(self):
            '''
            Determines moment of Inertia of a Solid Cylinder/Disk about Z Axis.-
            Format solid_vars: [mass, radius, length, bottom_z]
            '''
            solid_vars = ComponentType.components[ComponentType.ChangingSolidCylinder]
            mass = solid_vars[0]
            radius = solid_vars[1]
            return 0.5 * mass * (radius ** 2)
        
        def center_of_mass(self):
            '''
            Determines the center of mass of a hollow cylinder.
            Return value format: [z_coord, mass]
            '''
            hollow_vars = ComponentType.components[ComponentType.HollowCylinder]
            mass = hollow_vars[0]
            bottom_z = hollow_vars[3]
            length = hollow_vars[2]
            z_coord = bottom_z + length / 2
            return [z_coord, mass]

    class PointMass(Enum):
        # Component List Structure: [mass, bottom_z]
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
            lst = ComponentType.components[ComponentType.PointMass]
            return [lst[1], lst[0]]







