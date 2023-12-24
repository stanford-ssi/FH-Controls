import math
import numpy as np
from Vehicle.engine import Engine
from components import ComponentType


def total_moi_xy():
    """
    Calculates the total moment of inertia about the center of mass in the XY Plane, using the parallel axis theorem.  
    center_of_mass() outputs an individual component's [z_coord, mass]
    """
    tot_moi_xy = 0
    overall_com = calculate_com()
    for component, values in ComponentType.components.items():
        component: ComponentType = component
        difference = abs(overall_com - component.center_of_mass()[0])
        tot_moi_xy += component.moment_of_inertia_xy() + (component.center_of_mass()[1] * (difference ** 2))

    return tot_moi_xy

def total_moi_z():
    """
    Calculates the total moment of inertia about the center of mass in the Z Plane, about the central axis of the rocket. 
    """
    tot_moi_z = 0
    for component, values in ComponentType.components.items():
        component: ComponentType = component
        tot_moi_z += component.moment_of_inertia_z()

    return tot_moi_z

def fuel_proportion_remaining(rocket):
    """
    Given the integration of the throttle curve from t-initial to t-current,
    and the integration of the thrust curve from t-initial to t-final,
    returns the percent of fuel remaining.
    """
    return 1 - (rocket.engine.full_mass - rocket.engine.mass) / (rocket.engine.full_mass - rocket.engine.drymass)

def update_lengthwise_fuel(fuel_prop_remaining):
    """
    Updates the length of the two fuel cylinders that are emptying lengthwise (cylinder's height is decreasing)
    These are the components of type 'ChangingSolidCylinder'
    New Length = Current Length * Fuel Proportion Remaining
    ChangingSolidCylinder List Format: [mass, radius, current length, bottom_z, original_length]
    """
    for component, values in ComponentType.components.items():
        component: ComponentType = component
        if type(component) is ComponentType.ChangingSolidCylinder:
            original_length = ComponentType.components[component][-1]
            ComponentType.components[component][2] = original_length * fuel_prop_remaining

def update_radial_fuel(fuel_prop_remaining):
    """
    Updates the inner radius of the paraffin wax fuel cylinder that empties from the inside-out (Cylinder's inner radius is increasing as rocket rockets (:))
    These are the components of type 'ChangingHollowCylinder'
    New Inner Radius = SQRT(outer_radius ** 2 - (new_volume / (PI * height)))
    Changing Hollow Cylinder List Format: [mass, current_inner_radius, outer_radius, length, bottom_z, original_inner_radius]
    """
    for component, values in ComponentType.components.items():
        component: ComponentType = component
        if type(component) is ComponentType.ChangingHollowCylinder:
            cyl_vars = ComponentType.components[component]
            outer_rad = cyl_vars[2]
            original_inner_rad = cyl_vars[5]
            length = cyl_vars[3]

            new_volume = fuel_prop_remaining * math.pi * (outer_rad ** 2 - original_inner_rad ** 2) * length
            new_inner_rad = math.sqrt(outer_rad ** 2 - (new_volume / (math.pi * length)))
            cyl_vars[1] = new_inner_rad


#Throttle: Range 0 - 1 (Represents flow of oxygen out of bottom of rocket) Proportional
#We need to determine how depleted the fuel chambers are at any given time
#Length of "inner cylinders" change for two chambers, inner radius changes for last cylinder

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    
    ComponentType.components[ComponentType.HollowCylinder] = [1, 2, 3, 4, 5]
    ComponentType.components[ComponentType.SolidCylinder] = [1, 2, 3, 4]

    #For a given timestep:
    # STEPS (Updating MOI During Flight
    # 1. Calculate fuel proportion remaining
    current_fuel_remaining = fuel_proportion_remaining(rocket)
    # 2. Change fuel-component variables (using update)
    update_lengthwise_fuel(current_fuel_remaining)
    update_radial_fuel(current_fuel_remaining)
    # 3. Calculate new COM (using calculate_com()) - happens inside the functions of step 4
    # 4. Calculate new MOIs (total_moi_xy())
    current_xy_moi = total_moi_xy()
    current_z_moi = total_moi_z()


    
