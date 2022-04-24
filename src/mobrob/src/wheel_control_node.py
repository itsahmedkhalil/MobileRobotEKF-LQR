#!/usr/bin/env python3

import numpy as np
import rospy
import serial
import traceback 
import encoders
from pololu_drv8835_rpi import motors  	# MAX_SPEED is 480 (hard-coded)

# IMPORT the "encoders_and_motors" module so we can call its pieces as functions. 
import encoders_and_motors as encmot


# IMPORT the custom messages: 
# we import it "from" the ROS package we created it in (here "me439robot") with an extension of .msg ...
# and actually import the message type by name (here "ME439SensorsRaw" and others)
from mobrob_util.msg import ME439SensorsRaw, ME439WheelSpeeds, ME439MotorCommands, ME439WheelAngles, ME439WheelDisplacements
from std_msgs.msg import Float32MultiArray


#==============================================================================
# # Get parameters from rosparam
#==============================================================================
counts_per_encoder_revolution = rospy.get_param('/counts_per_encoder_revolution')  # 12.0
gear_ratio = rospy.get_param('/gear_ratio') # standard ME439 robot: 120.0   
wheel_radius = rospy.get_param('/wheel_diameter_actual')/2.0 # 0.030 # meters

encoder_update_rate = rospy.get_param('/encoder_update_rate_hz')

e0_direction_sign = rospy.get_param('/left_encoder_sign')
e1_direction_sign = rospy.get_param('/right_encoder_sign')
m0_direction_sign = rospy.get_param('/left_motor_sign')
m1_direction_sign = rospy.get_param('/right_motor_sign')

integral_error_max = rospy.get_param('/vel_integral_limit')
integral_resetting = rospy.get_param('/integral_resetting')
cmd_rate_of_change_max = rospy.get_param('/cmd_rate_of_change_max')
motor_command_max = rospy.get_param('/motor_command_max')

Kf0 = rospy.get_param('/vel_left_f')  # 
Kp0 = rospy.get_param('/vel_left_p') 
Ki0 = rospy.get_param('/vel_left_i')
Kd0 = rospy.get_param('/vel_left_d') 
Kf1 = rospy.get_param('/vel_right_f')  # 
Kp1 = rospy.get_param('/vel_right_p') 
Ki1 = rospy.get_param('/vel_right_i')
Kd1 = rospy.get_param('/vel_right_d') 

# Max encoder increment (full speed) - useful for eliminating errors
enc_increment_max = 1000


#==============================================================================
# # Set up a system to coordinate the motor closed-loop control
#==============================================================================
#   (Motor control is not ROS-friendly using ordinary messages due to delays.) 
def talker(): 
    # Launch a node called "sensing_and_wheel_control_node"
    rospy.init_node('sensing_and_wheel_control_node', anonymous=False)
        
#==============================================================================
#     # Create two Quadrature Encoder instances, one for each wheel 
#==============================================================================
    #   constructor function call: new_variable = encmot.quadrature_encoder(serial_string_identifier, counts_per_encoder_revolution, gear_ratio, wheel_radius, forward_encoder_rotation_sign)
    qe0 = encmot.quadrature_encoder("E0", counts_per_encoder_revolution,gear_ratio, wheel_radius, e0_direction_sign)
    qe1 = encmot.quadrature_encoder("E1",counts_per_encoder_revolution,gear_ratio, wheel_radius, e1_direction_sign)
    
#==============================================================================
#     # Create two motor controller instances, one for each wheel
#==============================================================================
    #   constructor function call: FPID_controller (): new_object = encmot.FPID_controller(motor,Kf, Kp,Ki,Kd, error_integral_limit=np.inf, integral_resetting = True, motor_command_max_rate_of_change=1500., forward_motor_command_sign=1)
    mc0 = encmot.FPID_controller(motor=motors.motor1, Kf=Kf0, Kp=Kp0, Ki=Ki0, Kd=Kd0, error_integral_limit = integral_error_max, integral_resetting = integral_resetting, motor_command_max_rate_of_change = cmd_rate_of_change_max, motor_command_max = motor_command_max, forward_motor_command_sign = m0_direction_sign)
    mc1 = encmot.FPID_controller(motor=motors.motor2, Kf=Kf1, Kp=Kp1, Ki=Ki1, Kd=Kd1, error_integral_limit = integral_error_max, integral_resetting = integral_resetting, motor_command_max_rate_of_change = cmd_rate_of_change_max, motor_command_max = motor_command_max, forward_motor_command_sign = m1_direction_sign)

#==============================================================================
#     # Here start a Subscriber to the "wheel_speeds_desired" topic.
#==============================================================================
    #   NOTE the Callback to the set_wheel_speed_targets function, which will update the setting of a motor controller that is called from the main loop below. 
    #   NOTE also the extra arguments to that callback: the Motor Encoders (both in a list)
    sub_wheel_speeds = rospy.Subscriber('/wheel_speeds_desired', ME439WheelSpeeds, set_wheel_speed_targets,[mc0,mc1])  
   
    encoder_update([qe0,qe1],[mc0,mc1])


#==============================================================================
# # Function to send updated Commands directly to the motor controllers.
#==============================================================================
#   These commands come from the Subscriber above. 
def set_wheel_speed_targets(msg_in, motor_controllers):
    motor_controllers[0].update_target_value(msg_in.v_left)
    motor_controllers[1].update_target_value(msg_in.v_right)
    
def encoder_update(quad_encoders, mot_controllers):
    # Create the publisher for the topic "/motor_commands", with message type "ME439MotorCommands"
    pub_motorcommands = rospy.Publisher('/motor_commands', ME439MotorCommands, queue_size=10)
    # Create the publisher for the topic "/robot_wheel_angles", with message type "ME439WheelAngles"    
    pub_robot_wheel_angles = rospy.Publisher('/robot_wheel_angles', ME439WheelAngles, queue_size = 10)
    # Create the publisher for the topic "/robot_wheel_displacements", with message type "ME439WheelDisplacements"    
    pub_robot_wheel_displacements = rospy.Publisher('/robot_wheel_displacements', ME439WheelDisplacements, queue_size = 10)
    # Create a publisher 
    pub_robot_wheel_vel = rospy.Publisher('/robot_wheel_vel', ME439WheelSpeeds, queue_size=10 )


    #Make messages to fill later.
    pub_motorcommands_message = ME439MotorCommands()
    pub_motorcommands_message.cmd0 = 0
    pub_motorcommands_message.cmd1 = 0

    robot_wheel_angles_message = ME439WheelAngles()
    robot_wheel_angles_message.ang_left = 0. 
    robot_wheel_angles_message.ang_right = 0. 

    robot_wheel_displacements_message = ME439WheelDisplacements()
    
    # robot_wheel_displacements_message.d_left = 0.
    # robot_wheel_displacements_message.d_right = 0.

    robot_wheel_vel_message = ME439WheelSpeeds()
    robot_wheel_vel_message.v_left = 0.
    robot_wheel_vel_message.v_right = 0.
    
    #Need elapsed time between loops even though it should be about 1/(encoder_update_rate)
    t0_previous = t1_previous = rospy.get_rostime()
    publishCount = 0;
    rosTimer = rospy.Rate(encoder_update_rate);
    while not rospy.is_shutdown():
        #read the encoders:
        [leftEnc,rightEnc] = encoders.readEncoders();
        
        #processing for left encoder.
        t0 = rospy.get_rostime()   # or time.time()
        dt0 = (t0-t0_previous).to_sec()
        t0_previous = t0 # store it for the next round
        #update PID loops
        quad_encoders[0].update(leftEnc, dt0)
        mot_controllers[0].update_current_value(quad_encoders[0].meters_per_second, dt0) 
        pub_motorcommands_message.cmd0 = int(mot_controllers[0].motor_command)
        robot_wheel_angles_message.ang_left = quad_encoders[0].radians
        robot_wheel_displacements_message.d_left = quad_encoders[0].meters
        robot_wheel_vel_message.v_left  = quad_encoders[0].meters_per_second
        robot_wheel_displacements_message.stamp  = rospy.Time.now()
        #processing for left encoder.
        t1 = rospy.get_rostime()   # or time.time()
        dt1 = (t1-t1_previous).to_sec()
        t1_previous = t1 # store it for the next round
        #update PID loops
        quad_encoders[1].update(rightEnc, dt1)
        mot_controllers[1].update_current_value(quad_encoders[1].meters_per_second, dt1)
        pub_motorcommands_message.cmd1 = int(mot_controllers[1].motor_command)
        robot_wheel_angles_message.ang_right = quad_encoders[1].radians
        robot_wheel_vel_message.v_right = quad_encoders[1].meters
        robot_wheel_displacements_message.d_right = quad_encoders[1].meters

        publishCount = publishCount +1;
        #only publish every 10 loops to avoid overloading ROS (should be ~10Hz)
        if (publishCount % 10 == 0):
            pub_motorcommands.publish(pub_motorcommands_message)
            # Publish a message to the "/robot_wheel_angles" topic
            pub_robot_wheel_angles.publish(robot_wheel_angles_message)
            # Publish a message to the "/robot_wheel_displacements" topi
            pub_robot_wheel_displacements.publish(robot_wheel_displacements_message)
            # publish
            pub_robot_wheel_vel.publish(robot_wheel_vel_message)
 
        rosTimer.sleep()

if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        traceback.print_exc()
        pass
    # Stop the wheels if this crashes or otherwise ends.     
    motors.setSpeeds(0,0)
