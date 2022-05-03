#!/usr/bin/env python3

import board
import adafruit_mpu6050
import rospy
import traceback 
import time
import numpy as np
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
    yaw_offset = []
    for i in range(200):
        yaw_offset.append(mpu.gyro[2])
    yaw_offset = np.average(yaw_offset)
    print("yaw offset:",yaw_offset)

    yaw_data = []
    # for i in range(3000):
    #     yaw_data.append(mpu.gyro[2]-yaw_offset)
    # yaw_std = np.std(yaw_data)
    # yaw_mean = np.mean(yaw_data)
    # print("Standard Deviation: ", yaw_std)
    # print("Mean: ", yaw_mean)

    while not rospy.is_shutdown():
        try:
            pub_gyro_msg.stamp = rospy.Time.now()           
            pub_gyro_msg.yaw = mpu.gyro[2] - yaw_offset
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
