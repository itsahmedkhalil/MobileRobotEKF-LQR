#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 26 12:12:05 2019

@author: pi
"""

from mpu6050 import mpu6050
import time
import numpy as np



def main():
    
    duration = 5.0  # seconds. How long should we display for? 
    
    imu = mpu6050(0x69)
    time_Start = time.time()
    
    # dummy code to print values for a while
    while time.time() < time_Start + duration:
        [accel, gyro, temp] = imu.get_all_data()
#        # this will print the data that are returned
#        print('a: {0}, w: {1}, T: {2}'.format(accel, gyro, temp))
        
        # Note how the data are stored in a Python Dictionary! You have to get them out by name: 
        ax = accel['x']
        ay = accel['y']
        az = accel['z']
        wx = gyro['x']
        wy = gyro['y']
        wz = gyro['z']
        temp = temp
#        # here we print them in a way that makes sense -each as a list
#        print('a: {0}, w: {1}, T: {2}'.format([ax, ay, az], [wx, wy, wz], [temp]))    
        
        # and here we print them as a bunch of numpy arrays: 
        a = np.array([ax, ay, az])
        w = np.array([wx, wy, wz])
        T = np.array([temp])
        print('a: {0}, w: {1}, T: {2}'.format(a, w, T))        

        
        time.sleep(0.5)
        
        
    
# the system literal "__name__" will be "__main__" if the user launches this file. 
# In that case, run the main program. 
# Otherwise it will be something related to whatever program called this one.     
if __name__ == "__main__":
    main()