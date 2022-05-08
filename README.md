# ME439MobileRobotEKF

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
#### Load the encoders on the Raspberry Pi
`$ cd ~/ME439MobileRobotEKF && sudoLoad`
#### Enter as root user
`$ sudo su `
#### Source and run the launch file
`$ src && roslaunch mobrob plant.launch`

### Run the project on the Linux Machine

#### Source the project
`$ cd ~/ME439MobileRobotEKF && source devel/setup.bash`
#### Run the launch file
`roslaunch mobrob controller.launch`