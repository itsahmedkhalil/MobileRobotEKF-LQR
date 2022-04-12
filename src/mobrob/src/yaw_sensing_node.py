#!/usr/bin/env python3

import numpy as np
import rospy
import serial
import traceback 

# IMPORT the custom messages: 
# we import it "from" the ROS package we created it in (here "me439robot") with an extension of .msg ...
# and actually import the message type by name (here "ME439SensorsRaw" and others)
from mobrob_util.msg import ME439SensorsRaw
from std_msgs.msg import Float32


#==============================================================================
# # Get parameters from rosparam
#==============================================================================
serial_data_port = '/dev/ttyUSB0'#rospy.get_param('/serial_data_port')
serial_baud_rate = 115200#rospy.get_param('/serial_baud_rate')



#=============================================================================
# # Set up a system to coordinate the motor closed-loop control
#==============================================================================
#   (Motor control is not ROS-friendly using ordinary messages due to delays.) 
def talker(): 
    # Launch a node called "sensing_and_wheel_control_node"
    rospy.init_node('yaw_sensing_node', anonymous=False)
        
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
    pub_sensors = rospy.Publisher('/yaw_gyro', Float32, queue_size=10)
    
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
    pub_gyro_msg = Float32()
    pub_gyro_msg.data = 0.0
    
    ser.flushInput()  # Flush again to avoid backlogs. 
    # MAIN LOOP to keep loading the message with new data. 
    while not rospy.is_shutdown():

        try: 
            # Here we read the serial port for a string that looks like "e0:123456", which is an Encoder0 reading. 
            # When we get a reading, update the associated motor command
            line = ser.readline().decode().strip() #blocking function, will wait until read entire line
#            print(line)
            line = line.split(":")
            data_type = line[0]
            data_value = float(line[1])
#            print(data_type)

            if data_type == 'A0':
                pub_gyro_msg.data = data_value
                newa0 = 1

#==============================================================================
#             # Publish a Message IFF it is full (i.e., all signals have been read including a5: a5 is the last we will always get)
#==============================================================================
            # Other logic could also be applied
            if newa0:
                #pub_gyro_msg.data = rospy.get_rostime()
                rospy.loginfo(pub_gyro_msg)
                # Publish a message to the "/sensors_data_raw" topic. 
                pub_sensors.publish(pub_gyro_msg)
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
