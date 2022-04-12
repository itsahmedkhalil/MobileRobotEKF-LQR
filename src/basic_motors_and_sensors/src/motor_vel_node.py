#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: Motor Node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk 
Updated 2021-02-09

motor_node.py
"""

import rospy
from std_msgs.msg import Float32MultiArray 
from pololu_drv8835_rpi import motors, MAX_SPEED  	# MAX_SPEED is 480 (hard-coded)

# Initialize the Node. This can happen here (to be executed as the script is interpreted by Python) or inside a function, but it must take place before any ROS communications take place. 
rospy.init_node('motor_vel_node',anonymous=False)

def listener(): 

    # Subscribe to the "wheel_command_left" topic
    ## 1st argument: topic; 2nd arg: message type; 3rd arg: callback function to with the incoming message will be passed)
    sub = rospy.Subscriber('/wheel_speeds', Float32MultiArray, set_velocity_commands) 
 
    #### CODE HERE ####
    # Add a Subscriber for the Right wheel
    #### END CODE ####
   
    rospy.spin()    # keep the node from exiting
    
    
# Callback for actually setting the command of the left wheel    
def set_velocity_commands(msg_in): 
    wheel_velocity_left = int(msg_in.data[0])
    motors.motor1.setSpeed(-wheel_velocity_left)
    wheel_velocity_right = int(msg_in.data[1])
    motors.motor2.setSpeed(wheel_velocity_right)
    
#### CODE HERE ####
# Add a callback for the Right wheel
#### END CODE ####    
    

# Section to start the execution, with Exception handling.     
if __name__ == "__main__":
    try: 
        listener()
    except rospy.ROSInterruptException: 
        motors.motor1.setSpeed(0)
        motors.motor2.setSpeed(0)
        pass
    
    motors.motor1.setSpeed(0)
    motors.motor2.setSpeed(0)
