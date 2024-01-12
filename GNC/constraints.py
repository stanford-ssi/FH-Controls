""" 
Will Add Further Documentation Here (Need To Agree On Formatting)


"""
from GNC.controlConstants import *

def throttle_checks(throttle, throttle_prev, ts):

    if abs((throttle - throttle_prev) / ts) > MAX_THROTTLE_RATE:
        throttle = throttle_prev + (MAX_THROTTLE_RATE * ts * ((throttle - throttle_prev) / abs(throttle - throttle_prev)))
        
    if throttle < MIN_THROTTLE:
        return MIN_THROTTLE
    if throttle > MAX_THROTTLE:
        return MAX_THROTTLE
    
    return throttle
    
def pos_checks(pos, pos_prev, ts):
        
    if abs((pos - pos_prev) / ts) > MAX_POS_RATE:
        pos = pos_prev + (MAX_POS_RATE * ts * ((pos - pos_prev) / abs(pos - pos_prev)))
        
    if pos < -MAX_POS: #m
        return -MAX_POS
    if pos > MAX_POS:
        return MAX_POS
    
    return pos
    

