#!/usr/bin/env python3

# =============================================================================
# Peter G. Adamczyk 
# 2018-10
# Updated 2021-02-26
# =============================================================================

import numpy as np


class quadrature_encoder:
    def __init__(self, serial_string_identifier, counts_per_encoder_revolution, gear_ratio, wheel_radius, forward_encoder_rotation_sign) :  
        # serial_string_identifier is e.g. "E0" or "E1"
        self.serial_string_identifier = serial_string_identifier  # does nothing, just an identifier that we hope the user will keep as a unique link to the sensor data streaming in. 
        self.counts_per_encoder_revolution = counts_per_encoder_revolution
        self.gear_ratio = gear_ratio
        self.wheel_radius = wheel_radius  # e.g. a wheel radius to get linear displacement and speed
        self.forward_encoder_rotation_sign = forward_encoder_rotation_sign
        
        self.counts_to_radians_multiplier = float(1)/self.counts_per_encoder_revolution / self.gear_ratio * 2*np.pi * self.forward_encoder_rotation_sign 
        
#        self.t = rospy.get_rostime()  # NOTE the ROS time function. Was time.time() for general python use. 
#        self.t_previous = self.t
        self.dt = 0.01 #(self.t - self.t_previous).to_sec()
        self.counts = 0
        self.counts_previous = 0
        self.radians = 0.
        self.meters = 0.
        self.counts_per_second = 0.
        self.radians_per_second = 0.
        self.meters_per_second = 0. 
        
        self.initializing = True  # this is a flag that will make "update" run twice (to set both position and per_second) the first time it is called. 

        
    def update(self, counts_measured, dt):
        self.dt = dt
        
        if self.initializing:
            self.counts_offset = counts_measured
            self.counts = counts_measured - self.counts_offset
            self.counts_previous = self.counts
            self.initializing = False
            return #    Prevent the rest of the function from running on the Initialization call. 
        
        self.counts_previous = self.counts
        self.counts = counts_measured - self.counts_offset
#        self.radians_previous = self.radians
        self.radians = self.counts * self.counts_to_radians_multiplier
#        self.meters_previous = self.meters 
        self.meters = self.radians * self.wheel_radius 
        
#        self.counts_per_second_previous = self.counts_per_second
        self.counts_per_second = float(self.counts - self.counts_previous)/self.dt
        # self.radians_per_second_previous = self.radians_per_second
        self.radians_per_second = self.counts_per_second * self.counts_to_radians_multiplier
        # self.meters_per_second_previous = self.meters_per_second
        self.meters_per_second = self.radians_per_second * self.wheel_radius 


        


# a general purpose FPID Velocity controller, to be instantiated in whatever way the user desires. 
class FPID_controller ():   
    def __init__(self, motor=[], Kf=0., Kp=1., Ki=0., Kd=0., error_integral_limit=np.inf, integral_resetting = True, motor_command_max_rate_of_change=100000., motor_command_max=480., forward_motor_command_sign=1): 
        self.motor = motor
        self.Kf = Kf
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.error_integral_limit = error_integral_limit
        self.integral_resetting = integral_resetting
        self.motor_command_max_rate_of_change = motor_command_max_rate_of_change
        self.motor_command_max = motor_command_max
        self.forward_motor_command_sign = forward_motor_command_sign
        
        # dummy values
        self.dt = 0.01     
        self.target = 0.
        self.error = 0.
        self.error_previous = 0.
        self.error_integral = 0.
        self.error_derivative = 0.
        
        self.motor_command = 0
        self.motor_command_previous = 0
        
        self.initializing = True
        
    def compute_error(self):
        self.error_previous = self.error 
        self.error = self.target - self.current
        
    def compute_error_integral(self):
        self.error_integral += self.error * self.dt
        # apply the error_integral_limit
        self.error_integral = np.clip(self.error_integral, -self.error_integral_limit, self.error_integral_limit)
#        # Special case of sitting still: do not allow Integral error to exist.
#        if float(self.target) == 0. :
#            self.error_integral = 0.
            
    def compute_error_derivative(self): 
        self.error_derivative = float(self.error - self.error_previous)/self.dt
        
    def compute_command(self): 
        ## With FeedForward Gain
        self.motor_command = self.forward_motor_command_sign * (self.Kf*self.target + self.Kp*self.error + self.Ki*self.error_integral + self.Kd*self.error_derivative)
    
    def limit_command_rate_of_change(self): 
        motor_command_rate_requested = (self.motor_command - self.motor_command_previous)/self.dt
        motor_command_rate = np.clip(motor_command_rate_requested, -self.motor_command_max_rate_of_change, self.motor_command_max_rate_of_change)
        self.motor_command = self.motor_command_previous + self.dt*motor_command_rate    

    def limit_command(self): 
        self.motor_command = np.clip(self.motor_command, -self.motor_command_max, self.motor_command_max) 
            
    def command_motor(self): 
        self.motor.setSpeed(int(self.motor_command))
        self.motor_command_previous = self.motor_command
        
    # Function to update the current value    
    def update_current_value(self, current_value, dt):
        self.current = current_value
        if self.initializing: 
            self.initializing = False
            return 
        self.dt = dt
        self.compute_error()
        self.compute_error_integral()
        self.compute_error_derivative()
        self.compute_command()
        self.limit_command_rate_of_change()
        self.limit_command()
        self.command_motor()
#        print([self.target, self.motor_command])
        
    def update_target_value(self, target):
        if target != self.target:
            self.target = target
            if self.integral_resetting:
                self.error_integral = 0.    # At one time we would reset the error integral with every command, but that doesn't work when you reset the target speed frequently. Now only do it with new commands, and only if using integral_resetting
#            self.update()     
#        print([self.target])
        
