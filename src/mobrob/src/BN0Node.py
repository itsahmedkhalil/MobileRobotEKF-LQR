#!/usr/bin/env python3

import board
import adafruit_bno055
import rospy
import traceback 
import time
import numpy as np
from mobrob_util.msg import IMU

class Orientation():
    
    def __init__(self):
        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        self.gyro_is_calibrated = False

    def calibrationStatus(self):
        try:
            while not self.gyro_is_calibrated:
                print(self.sensor.calibration_status)
                if self.sensor.calibration_status[1] == 3:

                    print("Gyro Calibration complete!")
                    self.gyro_is_calibrated = True
            print(self.sensor.offsets_gyroscope)
        except:
            print("Calibration failed")

    def angle(self):
        omega = self.sensor.gyro[2]
        return omega

if __name__ == '__main__':
    #calibrationStatus()
    BNO = Orientation()
    BNO.calibrationStatus()
    rospy.init_node('yaw_sensing_node', anonymous=False)
    pub_sensors = rospy.Publisher('/yaw_gyro', IMU, queue_size=2)
    rate = rospy.Rate(250)
    pub_gyro_msg = IMU()

    while not rospy.is_shutdown():
        try:
            pub_gyro_msg.stamp = rospy.Time.now()           
            pub_gyro_msg.yaw = BNO.angle()
            pub_sensors.publish(pub_gyro_msg)  
            
        except Exception: 
            pass
        rate.sleep()
