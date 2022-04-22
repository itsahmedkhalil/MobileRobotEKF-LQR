#!/usr/bin/env python3

import board
import adafruit_mpu6050
import rospy
import traceback 
import time
from mobrob_util.msg import IMU


i2c = board.I2C()  # uses board.SCL and board.SDA
mpu = adafruit_mpu6050.MPU6050(i2c)

def talker():

    rospy.init_node('yaw_sensing_node', anonymous=False)
    publisher()
    
def publisher():
# Create the publisher for the topic "/sensors_data_raw", with message type "ME439SensorsRaw"
    pub_sensors = rospy.Publisher('/yaw_gyro', IMU, queue_size=2)
    rate = rospy.Rate(300)
    pub_gyro_msg = IMU()

    while not rospy.is_shutdown():
        try:
            pub_gyro_msg.stamp = rospy.Time.now()           
            pub_gyro_msg.yaw = mpu.gyro[2]
            pub_sensors.publish(pub_gyro_msg)  
            
        except Exception: 
            pass
        rate.sleep()

if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        traceback.print_exc()
        pass
