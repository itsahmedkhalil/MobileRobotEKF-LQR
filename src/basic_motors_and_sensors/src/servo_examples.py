#!/usr/bin/python3

"""
ME 439 Intro to robotics

Move a set of 6 servos with a computer program. 
Peter Adamczyk, University of Wisconsin - Madison
Updated 2021-02-02
""" 

#from Adafruit_PWM_Servo_Driver import PWM
import numpy as np
import time
import traceback 


import Adafruit_PCA9685

# PWM Controller set-up
servo_pulse_frequency = 50
servo_pulse_duration_us = 1.0e6/servo_pulse_frequency   # microseconds for each pulse
servo_pulse_width_to_count_multiplier = 1./servo_pulse_duration_us*4095.0   # Counter is 12-bit, so 4096 levels (0-4095)

# Initialize the PWM device using the default address
pwm = Adafruit_PCA9685.PCA9685();

# Set the Frequency of all servo outputs
pwm.set_pwm_freq(servo_pulse_frequency)

# Shut down servos until they get a real command (temporary)
pwm.set_all_pwm(0,0)


# Function to command any servo with a given pulse width in microseconds
def command_servo(servo_number, pulse_width_us):
    pulse_width_count = int(round(pulse_width_us*servo_pulse_width_to_count_multiplier))
    pwm.set_pwm(servo_number, 0, pulse_width_count)



def interp_servo(angle_range,us_range,angle):
    # just use the same function arguments to call an Interpolation function np.interp
    return np.interp(angle,angle_range,us_range)    

def shutdown_servos():
    for ii in range(16):
        command_servo(ii, 0)


#%% Section to set up a nice trajectory 
def main():
    
# =============================================================================
#     Simple oscillation demo - a fixed number of cycles between two commands
#        # For two chosen Microsecond values 
#        # Command the servo
#        # Sleep for 1 second
#        # Switch values and repeat
# =============================================================================
    # which servo? 
    servo_num = 0
    us_values = np.array( [1400, 1600] )  # selected pair of angles to oscillate between
    for ii in range(5):   # 5 cycles
        command_servo(servo_num, us_values[0])   # Command 
        time.sleep(1)  # Sleep
        
        command_servo(servo_num, us_values[1])   # Command 
        time.sleep(1)  # Sleep
            
    
    
# =============================================================================
#     Interpolation setup - Calibrate angles
#     and use Angle commands instead of direct microseconds
# =============================================================================
    # Calibrated Angles for that servo - Increasing order
    angle_range = np.array([-90.,90.])
    # Calibrated microseconds that match 
    us_range = np.array([500,2500])
    
# =============================================================================
#     Interpolated oscillation demo - a fixed number of cycles between two angles
#        # For two chosen angles (40% and 60% of the way through the range above)
#        # Interpolate to find the microsecond command associated with the desired angle
#        # Command the servo
#        # Sleep for 1 second
#        # Switch angles and repeat
# =============================================================================
     # which servo? 
    servo_num = 0
    angles = angle_range[0] + np.array( [0.4, 0.6])*(max(angle_range)-min(angle_range) ) 
    for ii in range(5):
        us_command = interp_servo(angle_range,us_range,angles[0])  # Interpolate
        command_servo(servo_num, us_command)   # Command 
        time.sleep(1)  # Sleep
        
        us_command = interp_servo(angle_range,us_range,angles[1])  # Interpolate
        command_servo(servo_num, us_command)   # Command 
        time.sleep(1)  # Sleep
        
        

    
# =============================================================================
#     Trajectory demo
#     command a series of angles (a Trajectory)
#     And use the interpolation above to determine microsecond commands
# =============================================================================
    # duration of trajectory
    t_max = 5.
    # How often can you send a command to the servo? 
    dt_traj = 1./servo_pulse_frequency
    # Time array for the trajectory
    traj_time = np.linspace(0.,t_max,np.int(np.ceil(t_max/dt_traj)))
    n = len(traj_time)
    # ANGLES array for the trajectory 
    traj_ang = np.linspace(-90.,90.,n)

    # which servo? 
    servo_num = 0
    
    tstart = time.time()
    while time.time()-tstart < t_max:
        t_elapsed = time.time() - tstart
        # Find entries in the trajectory defined for times prior to (or equal to) the present time
        entries_past_or_present = traj_time <= t_elapsed
        # With the trajectory limited to those, select the last (latest) one (end of the array = index [-1])
        angle_to_command = traj_ang[entries_past_or_present][-1]
        # Interpolate to find the microsecond command associated with the desired angle
        us_command = interp_servo(angle_range,us_range,angle_to_command)
        # Command the servo
        command_servo(servo_num, us_command)
        # Report to the screen
        print(angle_to_command)
        # Sleep for a short time. 
        time.sleep(dt_traj/10.)
    
    print('Success')
    
    # Shut the servos down if the window closes 
    shutdown_servos()


if __name__=="__main__":
    try:
        main()

    except:
        traceback.print_exc()
        shutdown_servos()
        pass
