""" 
Will Add Further Documentation Here (Need To Agree On Formatting)


"""

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    #self-note for concern: if there are any errors with this code, it's like with the z_error and error portions, since z_error = error[2] and I'm unsure if this is all correct. 
    
    def getThrottle(self, current_throttle, error, dt):
        z_error = -1 * error[2]  # Extract the third element from the error array
        # P term
        p_term = self.kp * z_error
        
        # I term
        self.integral += z_error * dt
        i_term = self.ki * self.integral
        
        # D term
        derivative = (z_error - self.prev_error) / dt
        d_term = self.kd * derivative

        # Update throttle and previous error
        new_throttle = p_term + i_term + d_term

        self.prev_error = z_error
        return new_throttle
