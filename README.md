# ME439MobileRobotSimulation
#### Install Ros2 foxy ####
For Ubuntu 20.04: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

#### Build ####
`colcon build`

### Install Gazebo ###
`sudo apt install gazebo11 libgazebo11 libgazebo11-dev`

`sudo apt install ros-foxy-gazebo-ros-pkgs ros-foxy-cv-bridge`

#### Source Enviroment and Gazebo ####
`source install/setup.bash`

`export GAZEBO_MODEL_PATH=${PWD}/install/simulation/share/simulation/models`

`source /usr/share/gazebo/setup.sh`

#### Velocity publisher example command line ####
`ros2 topic pub /mobrob/cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"`

#### ROS1 Bridge ####
Tutorial: https://www.theconstructsim.com/ros2-qa-217-how-to-mix-ros1-and-ros2-packages/

`ros2 run ros1_bridge dynamic_bridge --bridge-all-topics`

#### Kill gazebo server if it crashes ####
`killall -9 gzserver`
