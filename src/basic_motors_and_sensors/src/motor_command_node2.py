#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: Motor Command Node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk 
Updated 2021-02-09

motor_command_node.py
ROS Node to get User Input of commands for wheel command (e.g. "wheel_command_left" - left wheel only at first) and publish them to a topic to set wheel commands in another node.  
"""

import rospy
from basic_motors_and_sensors.msg import WheelCommands
from std_msgs.msg import Float32MultiArray

# Initialize the Node. This can happen here (to be executed as the script is interpreted by Python) or inside a function, but it must take place before any ROS communications take place. 
rospy.init_node('motor_command_node',anonymous=False)

def talker_for_wheel_commands():
    
    # Set up a Publisher
    ## 1st argument: topic; 2nd arg: message type; 3rd arg: queue_size - use 1 if you only want the latest to be read. See https://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers)
    pub_wheel_commands = rospy.Publisher('wheel_commands',WheelCommands,queue_size=1)
    
    # Create the message object (empty at first) that will be populated and then published. Use the message type that was declared in the Publisher. 
    wheel_command_msg = WheelCommands()

    
    # Code for the specific functions of this Node: 
    # Here a while loop that gets user input of desired wheel command and publishes it. 
    # The condition on the "while" (rospy.is_shutdown()) evaluates whether ROS is in the process of shutting down, or has shut down. See https://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown 
    while not rospy.is_shutdown(): 
    
        # use the Python "input" command to get a value. Use the int() function to turn the string into an integer. 
        wheel_command_left, wheel_command_right = input('Enter wheel command left ,Enter wheel command right (-480 to +480) \n').split()
        wheel_command_left = int(wheel_command_left)
        wheel_command_right = int(wheel_command_right)

        # Check for good inputs and fix them if bad: 
        if wheel_command_left < -480: 
            wheel_command_left = -480
        elif wheel_command_left > 480: 
            wheel_command_left = 480
        
        if wheel_command_right < -480: 
            wheel_command_right = -480
        elif wheel_command_right > 480: 
            wheel_command_right = 480
        
        # Pack the message object with the current data.
        wheel_command_msg.commandL = wheel_command_left 
        wheel_command_msg.commandR = wheel_command_right
        
        # Publish the message. 
        pub_wheel_commands.publish(wheel_command_msg)
        rospy.sleep(5)
        wheel_command_msg.commandL = 0
        wheel_command_msg.commandR = 0
        pub_wheel_commands.publish(wheel_command_msg)

# Section to start the execution, with Exception handling.         
if __name__ == "__main__": 
    try: 
        talker_for_wheel_commands()
    except rospy.ROSInterruptException:
        pass
