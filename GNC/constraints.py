""" 
Will Add Further Documentation Here (Need To Agree On Formatting)


"""

def throttle_checks(throttle):
    if throttle < 0:
        return 0.1
    if throttle > 1:
        return 1.0
    return throttle
    
def pos_checks(pos):
    if pos < -0.2: #m
        return pos
    if pos > 0.2:
        return pos
    return pos
    

