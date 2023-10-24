

def getThrottle(state, idealstate):
    # EXTREMELY SHITTY. P control to move rocket back to location at timestep. Complete garbage, don't use for anything
    error = idealstate - state
