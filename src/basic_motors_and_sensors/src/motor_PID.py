
#title:lab 1 - ME 439 Intro to robotics


# =================================
#GOALS: Get the motors turning and following closed-loop velocity control patterns
# =================================


#------------imports----------------
import serial
from pololu_drv8835_rpi import motors, MAX_SPEED      # MAX_SPEED is 480 (hard-coded)
import time
import numpy as np
import traceback 

## Motor Demo
#for spd in range(0,481,1):
#    motors.motor1.setSpeed(spd)
#    time.sleep(0.01)
#    
#for spd in range(480,-481,-1):
#    motors.motor1.setSpeed(spd)
#    time.sleep(0.01)
#    
#for spd in range(-480,1,1): # stops before it gets to 1/100
#    motors.motor1.setSpeed(spd)
#    time.sleep(0.01)
#    
#print( 'Final Speed: {0}'.format(spd) )
#



#----------setup serial port to receive data from AlaMode--------------
ser = serial.Serial("/dev/ttyS0")  #serial port to alamode
ser.baudrate = 115200 
ser.bytesize = 8
ser.parity = 'N'
ser.stopbits = 1
ser.timeout = 1 # one second time out. 

# ser.open() causes error if run in Python3 - serial.Serial opened the port already in Python3.
if not ser.is_open: # check if it's not open (should be not open if running Python2)
    ser.open()  # necessary in Python2


# =============================================================================
# # Global Variables
# =============================================================================
# controller variables (used to compute error, error integral, etc.)
controller_error = 0.0
controller_error_previous = 0.0
controller_error_integral = 0.0
controller_error_derivative = 0.0


# =============================================================================
# # EDIT the Drive Train functions to make them represent the real motion of the robot.
# =============================================================================
#---------drive train---------------

def encoder_counts_to_output_shaft_radians(counts, sign):
    # GOAL: accept an input value in Encoder Counts and the sign of the motion...
    # and use the motor's parameters to return an angle in Radians
    angle_output_shaft_rad = counts*sign/12./120.*2*np.pi
    return angle_output_shaft_rad


def calculate_angvel(radPast, radPresent, dt = .01):
    # GOAL: accept inputs of past angle (radians), present angle (radians) ...
    # ... and elapsed time between them dt (here default value is 0.01 but can be overridden...
    # ... and return angular velocity in radians per second
    angvel_output_shaft_radpersec = (radPresent-radPast)/dt
#    print(angvel_output_shaft_radpersec)
    return angvel_output_shaft_radpersec



#---------PID functions-------------


def pidControl0(target, actual, dt):
    """
    takes a target value and an actual value, and dt, and computes PID controller output
    """
# =============================================================================
#     # MANIPULATE these constants to tune controller
# =============================================================================
    Kp = 50.
    Ki = 20.
    Kd = 0.

    # We are using Global variables just so they stay in memory. 
    # Here the word "global" gives this function (pidControl) access to the global variables that are initialized at the top of the file!
    global controller_error, controller_error_previous, controller_error_integral, controller_error_derivative
    
    # Compute the error: difference between the target and the actual current state. 
    controller_error = target - actual
    
    # Compute the integral of the error. One-step Euler integration.
    controller_error_integral = controller_error_integral + dt*controller_error
    
    # Compute the derivative of the error. One-step finite difference approximation. 
    controller_error_derivative = (controller_error - controller_error_previous)/dt
    
    # We are done with the previous value of error... set that variable to the current error to remember it for next timestep. 
    controller_error_previous = controller_error
    
    # Actually compute the Motor command
    command = Kp*controller_error + Ki*controller_error_integral + Kd*controller_error_derivative
    return command




#--------------main-----------------
def main():
    
# =============================================================================
#     Set up a time course of commands
# =============================================================================
    # duration of trajectory
    t_max = 5.
    # How often can you send a command ? 
    dt_traj = 0.01
    # Time array for the trajectory
    traj_time = np.linspace(0.,t_max,np.int(np.ceil(t_max/dt_traj)))
    n = len(traj_time)
    # Speed Commands array for the trajectory 
    traj_ang = np.linspace(0.,480.,n)
    
#    # Arrays to collect data. [Could also be used to set up pre-specified commands, e.g. by planning a sequence of "targets"]
#    time_array = np.zeros([n,2])    # "range" builds a row vector, so it has to be transposed to make a column vector.
#    target_array = np.zeros([n,2])  # Array to accumulate values the motor will be asked to hit. 
#    actual_array = np.zeros([n,2])  # Array to accumulate values the motor actually hits
#    control_voltage_array = np.zeros([n,2])     # Array of control voltages
#    theta_array = np.zeros([n,2])       # Array of motor angles
#    omega_array = np.zeros([n,2])       # Array of motor speeds

    
# =============================================================================
#    Below is a loop that's controlled by getting data from the serial port. 
#    That's where we get the state of each motor from the Encoders (all the "ser.____" stuff)
# =============================================================================

    # Initialize variables
    tstart = time.time()
    t0 = tstart
    t1 = tstart
    cnt0 = 0
    cnt1 = 0
    ang0 = 0
    ang1 = 0
    angvel0 = 0
    angvel1 = 0
    ind = 0     # Counter for logging data
    
    newdata = 0
    newE0 = 0
    newE1 = 0
    
    ## Clear the Serial Port so you don't have a big backlog of data 
    #ser.reset_input_buffer() 
    ser.flushInput()
    ser.readline()
    
#==============================================================================
#     Main Loop
#==============================================================================
    while time.time()-tstart < t_max:
        
        try:    # "try" runs code, but if there's a problem that throws an "exception"...
                # it will catch the exception without crashing the program
            # Here we read the serial port for a string that looks like "e0:123456", which is an Encoder0 reading. 
            # When we get a reading, update the associated motor command
            line = ser.readline().decode().strip() #ser.readline is a blocking function, will wait until reads entire line
            line = line.split(":")
    #        print(line[0])
            if line[0] == 'E0':    #only use it as an encoder0 reading if it is one. 
                t0_old = t0
                ang0_old = ang0
                cnt0 = int(line[1])    # Here is the actual encoder reading. 
                t0 = time.time()
                ang0 = encoder_counts_to_output_shaft_radians(cnt0,-1)    # you will use Motor and Gearbox Properties to determine the actual Angle. 
                angvel0 = calculate_angvel(ang0_old, ang0, t0-t0_old)
                print([cnt0,cnt1, ang0,ang1, angvel0,angvel1])
                newdata = 1
                newE0 = 1
            elif line[0] == 'E1':    #only use it as an encoder1 reading if it is one. 
                t1_old = t1
                ang1_old = ang1
                cnt1 = int(line[1])    # Here is the actual encoder reading. 
                t1 = time.time()
                ang1 = encoder_counts_to_output_shaft_radians(cnt1,1)    # you will use Motor and Gearbox Properties to determine the actual Angle. 
                angvel1 = calculate_angvel(ang1_old, ang1, t1-t1_old)
                print([cnt0,cnt1, ang0,ang1, angvel0,angvel1])
                newdata = 1
                newE1 = 1 
        
        except: # "except" catches any exceptions that occur within the "try" block
            traceback.print_exc() # Print any error and its traceback stack
            pass    # "pass" doesn't do anything. But we need a line here so the syntax works. (thought the Traceback also fixes it)


        if newdata:
            
            pass
            
# =============================================================================
#     #Controlling the motor here by un-commenting different lines.
# =============================================================================           
        #controller functions 
        #note - only use one type of control at a time!
        #motor1_setting = pidControl(target_position, current_position)
        if newE0:
            newE0 = 0
            target_velocity = 5.0
            current_velocity = angvel0
            target_position=20.0
            motor1_setting = pidControl0(target_velocity, current_velocity, t0-t0_old)
            #motor1_setting = pidControl(target_position, ang0, t0-t0_old)
            motors.motor1.setSpeed(np.int(motor1_setting))
#            print([target_position, ang0, motor1_setting])
            print([target_velocity, current_velocity,motor1_setting])
    
#        if newE1:
#            newE1 = 0
#            target_velocity = 5.0
#            current_velocity = angvel1
#            motor2_setting = pidControl1(target_velocity, current_velocity, t1-t1_old)
#            motors.motor2.setSpeed(np.int(motor2_setting))
#             print([target_velocity, current_velocity,motor2_setting])

        newdata = 0

# =============================================================================
#     #Controlling the motor here by un-commenting different lines.
# =============================================================================           
        #controller functions 
        #note - only use one type of control at a time!
        #motor1_setting = pidControl(target_position, current_position)
        #motor1_setting = pidControl(target_velocity, current_velocity)
        
#        #drive motor
#        motor1_setting = traj_ang[traj_time<time.time()][-1]  # command for the first motor (index 0)
#        motor2_setting = traj_ang[traj_time<time.time()][-1]  # command for the second motor (index 1)
#        motors.motor1.setSpeed(np.int(motor1_setting))
#        motors.motor2.setSpeed(np.int(motor2_setting))
#        
# =============================================================================
#     #
# =============================================================================  

#        #backup history variables
#        #radPast = radians
#        time_array[ind,:] = [t0,t1]    # "range" builds a row vector, so it has to be transposed to make a column vector.
#        target_array = np.zeros([n,2])  # Array to accumulate values the motor will be asked to hit. 
#        actual_array = np.zeros([n,2])  # Array to accumulate values the motor actually hits
#        motor1_setting_array = np.zeros([n,2])     # Array of control voltages
#        theta_array = np.zeros([n,2])       # Array of motor angles
#        omega_array = np.zeros([n,2])       # Array of motor speeds
#        
#        ind += 1




    # when it gets here the trajectory is over. 
    motors.motor1.setSpeed(0)
    motors.motor2.setSpeed(0)
    
    # When the loop has finished, plot the data
    from matplotlib import pyplot as plt
    plt.figure()
    line_handles = plt.plot(time_array, np.hstack( (target_array, actual_array, motor1_setting_array, theta_array, omega_array) ) )
    plt.legend(line_handles,["Target","Actual","M1 Command","Theta","Omega"])
    plt.xlabel("Time (s)")
#    plt.ylim([-10,25])
    plt.show()




#if __name__=="__main__":
#    try:
#        main()
#
#    except:
#        print('Broke') 
        
main()