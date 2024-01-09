""" 
Will Add Further Documentation Here (Need To Agree On Formatting)


"""
from GNC.controlConstants import *

def throttle_checks(throttle):
    if throttle < MIN_THROTTLE:
        return MIN_THROTTLE
    if throttle > MAX_THROTTLE:
        return MAX_THROTTLE
    return throttle
    
def pos_checks(pos):
    if pos < -MAX_POS: #m
        return -MAX_POS
    if pos > MAX_POS:
        return MAX_POS
    return pos
    

