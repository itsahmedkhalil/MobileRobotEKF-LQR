#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: Motor Node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk 
Updated 2021-02-09

motor_node.py
ROS Node to accept commands of "wheel_command_left" (left wheel only at first) and make the motors run on a robot using the Pololu DRV8835 Raspberry Pi Hat 
"""

from numpy import math
import rospy
from std_msgs.msg import Float32MultiArray 
from pololu_drv8835_rpi import motors, MAX_SPEED  	# MAX_SPEED is 480 (hard-coded)

# Initialize the Node. This can happen here (to be executed as the script is interpreted by Python) or inside a function, but it must take place before any ROS communications take place. 




class Distance:
    def __init__(self):

        # Subscribe to the "wheel_command_left" topic
        ## 1st argument: topic; 2nd arg: message type; 3rd arg: callback function to with the incoming message will be passed)
        self.encoder_left_old = 0
        self.encoder_right_old = 0
        self.sub = rospy.Subscriber('/encoder_values', Float32MultiArray , self.wheel_distance) 
        self.t_old = rospy.get_time()

        # self.forward_left_gain = -0.126
        # self.backward_left_gain = 0.138
        # self.forward_right_gain = 0.15
        # self.backward_right_gain = -0.1425
        self.wheel_radius = 3 #cm

        self.right_distance = 0
        self.left_distance = 0
        self.counter = 0

        #### CODE HERE ####
        # Add a Subscriber for the Right wheel
        #### END CODE ####
 
    #rospy.spin()    # keep the node from exiting
    

    def wheel_distance(self,msg_in):
        if self.counter == 0:
    
            self.encoder_left_init = int(msg_in.data[0])
            self.encoder_right_init = int(msg_in.data[1])
            self.counter = 1
            pass
        time = rospy.get_time()
        encoder_left = int(msg_in.data[0]) - self.encoder_left_init
        encoder_right = int(msg_in.data[1]) - self.encoder_right_init
        dt = time-self.t_old
        encoder_left_change = (encoder_left - self.encoder_left_old)
        encoder_right_change = (encoder_right - self.encoder_right_old)
        # encoder_left_rate = (encoder_left - self.encoder_left_old)/dt
        # encoder_right_rate = (encoder_right - self.encoder_right_old)/dt
        # if encoder_left_rate > 0:
        #     angular_left_rate = encoder_left_rate*self.forward_left_gain 
        # else:
        #     angular_left_rate = encoder_left_rate*self.backward_left_gain 

        # if encoder_right_rate > 0:
        #     angular_right_rate = encoder_right_rate*self.forward_right_gain 
        # else:
        #     angular_right_rate = encoder_right_rate*self.backward_right_gain 
        const = (720/(self.wheel_radius*math.pi))
        dx_right = encoder_right_change/const
        dx_left = encoder_left_change/const

        self.right_distance += dx_right 
        self.left_distance += dx_left

        self.encoder_left_old = encoder_left
        self.encoder_right_old = encoder_right 
        self.t_old = time  
        print(dx_right, dx_left, self.right_distance, self.left_distance)
    
# # Callback for actually setting the command of the left wheel    
# def set_wheel_commands(msg_in): 
#     encoder_left = int(msg_in.data[0])
#     motors.motor1.setSpeed(-wheel_command_left)
#     wheel_command_right = int(msg_in.data[1])
#     motors.motor2.setSpeed(wheel_command_right)
    
#### CODE HERE ####
# Add a callback for the Right wheel
#### END CODE ####    
    

# Section to start the execution, with Exception handling.     
if __name__ == "__main__":
    try: 
        rospy.init_node('wheel_distance_node',anonymous=False)
        distance = Distance()
        rospy.spin()
    except rospy.ROSInterruptException: 
        motors.motor1.setSpeed(0)
        motors.motor2.setSpeed(0)
        pass
    
    motors.motor1.setSpeed(0)
    motors.motor2.setSpeed(0)
