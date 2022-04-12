#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: Servo Command Node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk 
Updated 2021-02-16

servo_command_node.py
ROS Node to get User Input of servo microsecond commands (e.g. "/servo_command_0") and publish them to a topic to set servo movement in another node.  
"""

import rospy
# "traceback" is a library that lets you track down errors. 
import traceback
from std_msgs.msg import Int32

# Initialize the Node. This can happen here (to be executed as the script is interpreted by Python) or inside a function, but it must take place before any ROS communications take place. 
rospy.init_node('servo_command_node',anonymous=False)

def talker_for_servo_commands():
    
    # Set up a Publisher
    ## 1st argument: topic; 2nd arg: message type; 3rd arg: queue_size - use 1 if you only want the latest to be read. See https://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers)
    pub_servo_command_0 = rospy.Publisher('/servo_command_0',Int32,queue_size=1)
    
    # Create the message object (empty at first) that will be populated and then published. Use the message type that was declared in the Publisher. 
    servo_command_0_msg = Int32()
    
    #### CODE HERE ####
    # Add a Publisher for any more servos
    #### END CODE ####
    
    
    # Code for the specific functions of this Node: 
    # Here a while loop that gets user input of desired servo command and publishes it. 
    # The condition on the "while" (rospy.is_shutdown()) evaluates whether ROS is in the process of shutting down, or has shut down. See https://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown 
    while not rospy.is_shutdown(): 
    
        # use the Python "input" command to get a value. Use the int() function to turn the string into an integer. 
        servo_command_0= int(input('Enter servo command 0 (roughly 500 to 2500) \n'))
            
        # Pack the message object with the current data.
        servo_command_0_msg.data = servo_command_0
        
        # Publish the message. 
        pub_servo_command_0.publish(servo_command_0_msg)
        
        
        #### CODE HERE ####
        # Add a input and publishing for any other servos
        #### END CODE ####





# Section to start the execution, with Exception handling.         
if __name__ == "__main__": 
    try: 
        talker_for_servo_commands()
    except : 
        traceback.print_exc()
        pass
