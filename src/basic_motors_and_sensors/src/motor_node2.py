#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: Motor Node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk 
Updated 2021-02-09

motor_node.py
ROS Node to accept commands of "wheel_command_left" (left wheel only at first) and make the motors run on a robot using the Pololu DRV8835 Raspberry Pi Hat 
"""

import rospy
from basic_motors_and_sensors.msg import WheelCommands
from std_msgs.msg import Float32MultiArray 
from pololu_drv8835_rpi import motors, MAX_SPEED  	# MAX_SPEED is 480 (hard-coded)

# Initialize the Node. This can happen here (to be executed as the script is interpreted by Python) or inside a function, but it must take place before any ROS communications take place. 
rospy.init_node('motor_node',anonymous=False)

def listener(): 

    # Subscribe to the "wheel_command_left" topic
    ## 1st argument: topic; 2nd arg: message type; 3rd arg: callback function to with the incoming message will be passed)
    sub = rospy.Subscriber('/wheel_commands', WheelCommands, set_wheel_commands) 
 
    #### CODE HERE ####
    # Add a Subscriber for the Right wheel
    #### END CODE ####
   
    rospy.spin()    # keep the node from exiting
    
    
# Callback for actually setting the command of the left wheel    
def set_wheel_commands(msg_in): 
    wheel_command_left = int(msg_in.commandL)
    motors.motor1.setSpeed(-wheel_command_left)
    wheel_command_right = int(msg_in.commandR)
    motors.motor2.setSpeed(wheel_command_right)
    
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
