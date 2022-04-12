#!/usr/bin/env python3

import numpy as np
import rospy
import serial
import traceback 

# IMPORT the custom messages: 
# we import it "from" the ROS package we created it in (here "me439robot") with an extension of .msg ...
# and actually import the message type by name (here "ME439SensorsRaw" and others)
from mobrob_util.msg import ME439SensorsRaw


#==============================================================================
# # Get parameters from rosparam
#==============================================================================
serial_data_port = '/dev/ttyUSB0'#rospy.get_param('/serial_data_port')
serial_baud_rate = 57600#rospy.get_param('/serial_baud_rate')

# Max encoder increment (full speed) - useful for eliminating errors
enc_increment_max = 1000


#=============================================================================
# # Set up a system to coordinate the motor closed-loop control
#==============================================================================
#   (Motor control is not ROS-friendly using ordinary messages due to delays.) 
def talker(): 
    # Launch a node called "sensing_and_wheel_control_node"
    rospy.init_node('sensing_node', anonymous=False)
        
#==============================================================================
#     # Start the loop that listens and publishes. 
#==============================================================================
    # Give it an argument of the [qe0,qe1] encoders and the [mc0,mc1] motor controllers so it can update them all. 
    serial_port_publisher()
    
#==============================================================================
# # Main loop function that listens to the serial port, calls motor control, and publishes the sensor data
#==============================================================================
def serial_port_publisher():
# Create the publisher for the topic "/sensors_data_raw", with message type "ME439SensorsRaw"
    pub_sensors = rospy.Publisher('/sensors_data_raw', ME439SensorsRaw, queue_size=10)
    
    # Data comes in on the Serial port. Set that up and start it. 
    #----------setup serial--------------
    ser = serial.Serial(serial_data_port)  #serial port to Alamode or Arduino
    ser.baudrate = serial_baud_rate     # 115200 or *57600*
    ser.bytesize = 8
    ser.parity = 'N'
    ser.stopbits = 1
    ser.timeout = 1 # one second time out. 

    ser.flushInput()
    ser.readline()
    
    
    # Declare the message that will go on that topic. 
    # Here we use one of the message name types we Imported, and add parentheses to call it as a function. 
    # We could also put data in it right away - here initializing to zero. 
    pub_sensors_message = ME439SensorsRaw()
    pub_sensors_message.e0 = 0
    pub_sensors_message.e1 = 0
    pub_sensors_message.a0 = 0
    pub_sensors_message.a1 = 0
    pub_sensors_message.a2 = 0
    pub_sensors_message.a3 = 0
    pub_sensors_message.a4 = 0
    pub_sensors_message.a5 = 0
    pub_sensors_message.u0 = 0
    pub_sensors_message.u1 = 0
    pub_sensors_message.u2 = 0
    
    ser.flushInput()  # Flush again to avoid backlogs. 
    # MAIN LOOP to keep loading the message with new data. 
    while not rospy.is_shutdown():
        # set all the "new data" variables to zero. 
        newe0 = 0
        newe1 = 0
        newa0 = 0
        newa1 = 0
        newa2 = 0
        newa3 = 0
        newa4 = 0
        newa5 = 0
        newu0 = 0
        newu1 = 0
        newu2 = 0
#        new_data_packet = 0
        try: 
            # Here we read the serial port for a string that looks like "e0:123456", which is an Encoder0 reading. 
            # When we get a reading, update the associated motor command
            line = ser.readline().decode().strip() #blocking function, will wait until read entire line
#            print(line)
            line = line.split(":")
            data_type = line[0]
            data_value = float(line[1])
#            print(data_type)
#            print(line)
            if data_type == 'E0':    #only use it as an encoder0 reading if it is one. 
                continue #don't process encoders in here anymore.. moved to wheel_control_node
            elif data_type == 'E1':    #only use it as an encoder1 reading if it is one. 
                continue #don't process encoders in here anymore.. moved to wheel_control_node
            elif data_type == 'A0':
                pub_sensors_message.a0 = data_value
                newa0 = 1
#                print(a0)
#                print("A0 = {0}").format(a0)
            elif data_type == 'A1':
                pub_sensors_message.a1 = data_value
                newa1 = 1
#                print(a1)
            elif data_type == 'A2':
                pub_sensors_message.a2 = data_value
                newa2 = 1
#                print(a2)
            elif data_type == 'A3':
                pub_sensors_message.a3 = data_value
                newa3 = 1
#                print(a3)
            elif data_type == 'A4':
                pub_sensors_message.a4 = data_value
                newa4 = 1
#                print(a4)
            elif data_type == 'A5':
                pub_sensors_message.a5 = data_value
                newa5 = 1
#                print(a5)
            elif data_type == 'U0':
                pub_sensors_message.u0 = data_value
                newu0 = 1
#                print(u0)
            elif data_type == 'U1':
                pub_sensors_message.u1 = data_value
                newu1 = 1
#                print(u1)
            elif data_type == 'U2':
                pub_sensors_message.u2 = data_value
                newu2 = 1
#                print(u2)
            
#==============================================================================
#             # Publish a Message IFF it is full (i.e., all signals have been read including a5: a5 is the last we will always get)
#==============================================================================
            # Other logic could also be applied
            if newa0:
                pub_sensors_message.t = rospy.get_rostime()
                rospy.loginfo(pub_sensors_message)
                # Publish a message to the "/sensors_data_raw" topic. 
                pub_sensors.publish(pub_sensors_message)
                # Log the info (optional)
                rospy.loginfo(pub_sensors)       
                
        
        except Exception: 
            pass

if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        traceback.print_exc()
        pass
