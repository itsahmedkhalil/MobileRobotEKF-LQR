#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: sensors_processor node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk 
Updated 2021-09-21

sensors_processor.py
ROS Node to a receive sensor data on a topic ("sensors_A0" or other) and process it, then publish the processed version to another topic ("sensors_A0_processed" or similar)  
"""


from this import d
import rospy
from std_msgs.msg import Int32, Float32

# First create one or more publishers for the topics that will hold the processed data. 
# They are created outside the "listener" function so that they are "in scope" (i.e. readable) when they are eventually used within the Callback function(s) invoked by the Subscriber(s) upon receipt of incoming messages (here the callback is "cal_and_pub_A0")
pub_A0_proc = rospy.Publisher('/sensors_A0_proc',Float32,queue_size=1)
A0_proc = Float32()

# This is the central code that will run in this node: setting up a node with a Subscriber object and associated Callback. 
def sensors_listener(): 
    # Initialize the Node
    rospy.init_node('sensors_processor', anonymous=False)
    
    # Set up a listener for the topic you want
    # Remember to name a "callback" function (must define below) to handle the data
    sub_A0 = rospy.Subscriber('/sensors_A0',Int32, cal_and_pub_A0)
    
    # spin() to prevent the function from exiting
    rospy.spin()
    
    
# Callback function, which will be called with incoming message data when messages are received by the Subscriber above. 
def cal_and_pub_A0(msg_in):
    analog_level = float(msg_in.data)
    
    # calibrate the relationship between the signal that comes in and the signal in real units
    # This can be from a Data Sheet, or from measurements you make yourself.
    
    # Example: MaxBotix LV-EZ02 says: 
    # Distance in Inches = Volts / (Vcc/512). 
    # For A0, Vcc is 3.3 Volts and signals are in 2^10 levels from 0 to Vcc
    # Therefore (Distance in Meters) = 0.0254 (m/in)*(distance in Inches)
    
##### UPDATE THESE EQUATIONS
    analog_volts = analog_level * (1.0/1.0)
    distance_meters = analog_volts / (1.0/1.0) * 1.0
    print(distance_meters)
#####
    
    # Create a Message that will hold the out-going data
    
    
    # Pack the message with the processed data
    A0_proc.data = distance_meters#distance_meters 
    
    # Publish the newly packed Message
    pub_A0_proc.publish(A0_proc)
    
    
    
if __name__ == '__main__':
    try: 
        sensors_listener()
    except rospy.ROSInterruptException: 
        pass

    