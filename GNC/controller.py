""" 
Will Add Further Documentation Here (Need To Agree On Formatting)


"""

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0

    #self-note for concern: if there are any errors with this code, it's like with the z_error and error portions, since z_error = error[2] and I'm unsure if this is all correct. 
    
    def control(self, error, prev_error, dt, type):
        # P term
        p_term = self.kp * error
        
        # I term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # D term
        derivative = (error - prev_error) / dt
        d_term = self.kd * derivative

        # Update throttle and previous error
        new = p_term + i_term + d_term
        if type == 'throttle':
            new = self.throttle_checks(new)
        if type == 'posx' or type == 'posy':
            new = self.pos_checks(new)

        return new

    def throttle_checks(self, throttle):
        if throttle < 0:
            return 0.1
        if throttle > 1:
            return 1.0
        return throttle
    
    def pos_checks(self, pos):
        if pos < -0.2: #m
            return pos
        if pos > 0.2:
            return pos
        return pos
    

