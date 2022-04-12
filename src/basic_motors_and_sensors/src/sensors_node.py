#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: sensors_node Node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk 
Updated 2021-09-21

sensors_node.py
ROS Node to a continually read data from a Serial port ('/dev/ttyUSB0' (Arduino Nano on USB) and publish it on a topic ("sensors_A0" or other)   
"""


import rospy
# Import "serial" to get data from the AlaMode
import serial   
# "traceback" is a library that lets you track down errors. 
import traceback 
# Import the message types we will need
from std_msgs.msg import Float32


# Publish sensors data at the rate it comes in
# Here we publish: 
#    Analog value of interest (levels) 
# 
# Could also publish Encoders (E0 and E1, (in "counts")), more Analogs (A0-A7, in "levels" 0-1023) and Ultrasound (in microseconds till echo) readings
# Here we publish each in a separate topic. 
# In the long-term it would be better to publish them together.  

# This is the central code: set up a Node, set up a Publisher(s)
def sensors_reader(): 
    # Launch a node called "sensors_node"
    rospy.init_node('sensors_node', anonymous=False)

    # Create the publishers. Name each topic "sensors_##", with message type "Int32" 
    # (because that's what is received over the serial port)
    # Note the queue_size=1 statement: don't let it develop a backlog! 
    
    # Example: A0
    pub_A0 = rospy.Publisher('/sensors_yaw', Float32, queue_size=1)
    
    # Other examples: 
    # pub_E0 = rospy.Publisher('/sensors_E0', Int32, queue_size=1)
    # pub_E1 = rospy.Publisher('/sensors_E1', Int32, queue_size=1)
    
    # Declare the message that will go on the topic. 
    # Here we use one of the message name types we Imported, and add parentheses to call it as a function. 
    # We put data in it using the .data field of the message.
    msg_A0 = Float32()
    
    # Other examples
#    msg_E0 = Int32()
#    msg_E1 = Int32()
    

# Data comes in on the Serial port. Set that up and start it. 

    #----------setup serial--------------
    ser = serial.Serial('/dev/ttyUSB0')  #serial port to alamode is /dev/ttyS0. # port to Arduino Nano is /dev/ttyUSB0 
    ser.baudrate = 57600 
    ser.bytesize = 8
    ser.parity = 'N'
    ser.stopbits = 1
    ser.timeout = 1 # one second time out. 

    data = 0
    ser.flush()  # Flush any data currently on the port
    ser.readline()



    # MAIN LOOP to keep loading the message with new data. 
    # NOTE that at the moment the data are coming from a separate thread, but this will be replaced with the serial port line reader in the future. 
    while not rospy.is_shutdown():

        try: 
            # Here we read the serial port for a string that looks like "e0:123456", which is an Encoder0 reading. 
            # When we get a reading, update the associated motor command
            line = ser.readline().decode().strip() #blocking function, will wait until read entire line
#            print(line)
            line = line.split(":")
            # Element 0 of "line" will be a string that says what the data are: 
            data_type = line[0]
            # Element 1 of "line" will be the value, an Integer
            data_value = float(line[1])
#            print(data_type)
#            print(line)
            if data_type == 'A0':
                #data += data_value*(1/9) 
                msg_A0 = data_value	# Analog reading 
                pub_A0.publish(msg_A0)
            else:
                continue
            
        
        except Exception:
            traceback.print_exc()
            pass
            



if __name__ == '__main__':
    try: 
        sensors_reader()
    except rospy.ROSInterruptException: 
        pass
