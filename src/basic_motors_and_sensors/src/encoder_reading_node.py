#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: Encoder Reading Node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk, with gratitude to Josh Tabor. 
Updated 2021-09-21

encoder_reading_node.py
ROS Node to get encoder values read by the Raspberry Pi through a Linux Kernel Module and publish them to topics that can be processed in another node.  
"""

import numpy as np  # We often do numerical manipulations, and Numpy makes it easier. 
import rospy        # ROSPY is the Python module that talks to ROS (sets up nodes, publishers, subscribers, etc.)
import traceback    # Traceback allows tracing of errors
import encoders     # Interface to the Linux Kernel Module for counting Encoders. 
from std_msgs.msg import Float32MultiArray  # "std_msgs.msg" is from ROS, and Int32 is a ROS message type. 


# Initialize the Node. This can happen here (to be executed as the script is interpreted by Python) or inside a function, but it must take place before any ROS communications take place. 
# "anonymous=True" would allow more than one of it; we do not want that. 
rospy.init_node('encoder_reading_node', anonymous=False)


# Define a function that will contain the main functions of our node. 
def talker(): 
    
    # Set up a Publisher for encoder values.
    # 1st argument: topic; 2nd arg: message type; 3rd arg: queue_size - use 1 if you only want the latest to be read. See https://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers) 
    # pub_encL = rospy.Publisher('/encoderLeft', Int32, queue_size=1)
    
    # # Create the message object (empty at first) that will be populated and then published. Use the message type that was declared in the Publisher. 
    # msg_encL = Int32()
    
    # #### CODE HERE ####
    # # 1st argument: topic; 2nd arg: message type; 3rd arg: queue_size - use 1 if you only want the latest to be read. See https://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers) 
    # pub_encR = rospy.Publisher('/encoderRight', Int32, queue_size=1)
    
    # # Create the message object (empty at first) that will be populated and then published. Use the message type that was declared in the Publisher. 
    # msg_encR = Int32()    #### END CODE ####
    pub_enc = rospy.Publisher('/encoder_values', Float32MultiArray, queue_size =1)
    msg_enc =  Float32MultiArray()
    
    #### Code for the specific functions of this Node: 
    
    # Set up a ROS Rate Timer to control the execution frequency of the loop. 
    rosTimer = rospy.Rate(10);  # Frequency in Hz
    
    # Here a while loop that gets the encoder values and publishes them. 
    # The condition on the "while" (rospy.is_shutdown()) evaluates whether ROS is in the process of shutting down, or has shut down. See https://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown     
    while not rospy.is_shutdown():
        #read the encoders:
        [leftEnc,rightEnc] = encoders.readEncoders() # They return as Ints

        # Pack a message for the Left encoder and Publish it to the "/encoderLeft" topic
        # msg_encL.data = leftEnc  # leftEnc will come back as an Int, so packs directly into Int32 message type. 
        # pub_encL.publish(msg_encL)  # Actually publish it in ROS. 

        # #### CODE HERE ####
        # # Pack a message for the Left encoder and Publish it to the "/encoderLeft" topic
        # msg_encR.data = rightEnc  # leftEnc will come back as an Int, so packs directly into Int32 message type. 
        # pub_encR.publish(msg_encR)  # Actually publish it in ROS.         #### END CODE ####

        msg_enc.data = [leftEnc,rightEnc]
        pub_enc.publish(msg_enc)

        rosTimer.sleep()





# Section to start the execution, with Exception handling.   
if __name__ == '__main__':
    try:    # try:except catches errors. 
        talker()  # this line calls the "talker" function defined above. 
    except rospy.ROSInterruptException:  
        traceback.print_exc()  # If an error occurs, this will print a traceback so you can find out where it happened. 
        pass

