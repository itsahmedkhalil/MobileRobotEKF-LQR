# ME 439 Final Project: Extended Kalman Filter and Linear Quadratic Regulator on Differential Mobile Robot for Trajectory Tracking
## Project Overview
### Description of Project

Indoor state estimation has been a challenging field of research due to the lack of sensors, such as GPS. Our project is to improve the current state estimation method, dead reckoning, by integrating an IMU sensor and fusing it with wheel encoders using an Extended Kalman Filter. In addition to obtaining a more accurate measurement for orientation, the IMU could still determine the robot’s orientation when the wheels cannot. If the wheels slip or move over non-ideal terrain, the robot’s orientation could still be determined.   
	
The project also aimed to implement a Linear Quadratic Regulator (LQR) controller and to compare it to the Proportional Integral Derivative (PID) controller designed in class. To aid with tuning parameters and verifying algorithms, various techniques were used to speed up development, such as using simulation and visualization software like Gazebo and rviz. 

Click on the photo below to be redirected to a short video of the project being run in real time.
[![IMAGE_ALT](/images/mobileRobot.jpeg)](https://www.youtube.com/watch?v=iVwEiv4ZqnQ)

### Equipment Used
- Encoders: [**Pololu 3081 Encoder kit**](https://www.pololu.com/product/3081)
- 6DOF IMU: [**Adafruit MPU6050**](https://makeradvisor.com/tools/mpu-6050-3-axis-accelerometer-and-gyroscope-sensor/)
- 9DOF IMU: [**Adafruit BNO055**](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor)
- Wheels: [**Pololu 1423 wheel (60 mm)**](https://www.pololu.com/product/1423)
- Caster: [**Pololu 2691 Ball Caster**](https://www.pololu.com/product/2691)
- Arduino Nano: [**Arduino Nano with ATMega328**](http://store.arduino.cc/products/arduino-nano)
- Frame: Laser cut at UW Madison Makerspace 

**Note**: A Rasberry Pi 4 was used for data sensor acquistion from the encoders and IMU, however, all computations were done off-board on an external Linux machine for more computational power. 

### Collaborators
This project was created by [Ahmed Khalil](https://itsahmedkhalil.github.io/) and [Mohamed Safwat](https://mohsafwat23.github.io/) as part of their final project for ME 439: Introduction to Robotics at the University of Wisconsin-Madison. This project only had a duration of 3 weeks so the team would like to revisit this project in the future to finalize their findings.
## Mobile Robot Setup on Raspberry Pi

### Clone the code from the GitHub repository
`$ git clone https://github.com/mohsafwat23/ME439MobileRobotEKF.git`

#### Build the project
`$ cd ME439MobileRobotEKF && catkin_make`

### Create bashrc commands to run the project easily

#### Edit the bashrc file 
`$ nano ~/.bashrc`

#### Add the following commands to the bashrc file

`sudoLoad() {
        cd ~/ClonedRepos/RaspberryPiKernelEncoder && sudo make load && cd ~/ME439MobileRobotEKF/
}`

and 

`src() {
        source devel/setup.bash
}`

#### Save and exit
` cntrl-s + cntrl-x`
#### Source the bashrc file
`$ source ~/.bashrc`

### Install ROS libraries for sensors

#### Install MPU6050 library: https://learn.adafruit.com/mpu6050-6-dof-accelerometer-and-gyro/python-and-circuitpython
`$ sudo pip3 install adafruit-circuitpython-mpu6050`

#### Install BNO055 library: https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/python-circuitpython
`$ sudo pip3 install adafruit-circuitpython-bno055`

## Off-board Controller Setup on Linux Machine

### Clone the code from the GitHub repository
`$ git clone https://github.com/mohsafwat23/ME439MobileRobotEKF.git`

#### Build the project
`$ cd ME439MobileRobotEKF && catkin_make`

## Setup ROS Networking
### Use this link to setup ROS Networking: https://razbotics.wordpress.com/2018/01/23/ros-distributed-systems/


## Running the project

### Run the project on the Raspberry Pi
#### Load the encoders on the Raspberry Pi (Only needs to be run once everytime the Raspberry Pi is powered on)
`$ cd ~/ME439MobileRobotEKF && sudoLoad`
#### Enter as root user
`$ sudo su`
#### Source and run the launch file
`$ src && roslaunch mobrob plant.launch`

### Run the project on the Linux Machine

#### Source the project
`$ cd ~/ME439MobileRobotEKF && source devel/setup.bash`
#### Run the launch file
`$ roslaunch mobrob controller.launch`

### Modifications to the controller
#### To switch between using the dead reckoning state estimator and the EKF, change the following line in the controller.launch file:

remap the topic names from /robot_pose_estimated to /robot_pose_ekf