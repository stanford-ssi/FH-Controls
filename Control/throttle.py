

def getThrottle(current_throttle, error):
    # EXTREMELY SHITTY. P control to move rocket back to location at timestep. Complete garbage, don't use for anything
    kp = 1
    z_error = error[2]
    new_throttle = current_throttle + (z_error * kp)
    return new_throttle



