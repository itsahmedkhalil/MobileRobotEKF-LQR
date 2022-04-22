import time
import board
import adafruit_mpu6050

i2c = board.I2C()  # uses board.SCL and board.SDA
mpu = adafruit_mpu6050.MPU6050(i2c)

t = time.time()
while True:
    print(mpu.gyro)
    time.sleep(0.001)