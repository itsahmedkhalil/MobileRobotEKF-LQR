#!/usr/bin/python3
# got info from http://blog.bitify.co.uk/2013/11/reading-data-from-mpu-6050-on-raspberry.html

import smbus
import math
import matplotlib
import matplotlib.pyplot as plt
import time

############# Code for IMU
# Device Address
# This is the address value read via the i2cdetect command. 
# Will be 0x68 (default) or 0x69 (if AD0 is connected to "high"). 
address = 0x69       

# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
 
def read_byte(adr):
    return bus.read_byte_data(address, adr)
 

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val
 
def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val
 
def dist(a,b):
    return math.sqrt((a*a)+(b*b))
 
def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)
 
def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)
    
# Access the I2C bus
bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards

# Now wake the 6050 up as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)

def read_imu():
    gyro_x = read_word_2c(0x43) / 131
    gyro_y = read_word_2c(0x45) / 131
    gyro_z = read_word_2c(0x47) / 131   

    accel_x = read_word_2c(0x3b) / 16384.0
    accel_y = read_word_2c(0x3d) / 16384.0
    accel_z = read_word_2c(0x3f) / 16384.0
        
    x_rotation = get_x_rotation(accel_x, accel_y, accel_z)
    y_rotation = get_y_rotation(accel_x, accel_y, accel_z)
    # print('gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, x rotation, y rotation: ', gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, x_rotation, y_rotation)
    return (gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, x_rotation, y_rotation)

#########################################################

#########################################################
# Testing and Plotting 5sec of data for x and y rotations in IMU
x=[]; y=[]; t =[]; imu_data=[]
for i in range(100): # first value is not included
	imu_data=read_imu()
	print(imu_data)
	x.append(imu_data[6])
	y.append(imu_data[7])
	t.append(0.05*i)
	time.sleep(0.05)
	
plt.plot(t,x,t,y)
plt.xlabel('Time(s)')
plt.ylabel('X and Y Rotation(degrees)')
plt.title('imu_oscillations')
plt.gca().legend(('x','y'))
plt.show()
#plt.savefig('imu_oscillations.png')
plt.close()
	















