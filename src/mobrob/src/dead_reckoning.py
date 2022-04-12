#!/usr/bin/env python3

# =============================================================================
# Peter G. Adamczyk 
# Updated 2021-02-27
# =============================================================================


import rospy
import numpy as np
import traceback 
from geometry_msgs.msg import Pose2D
from mobrob_util.msg import ME439WheelDisplacements

#==============================================================================
# # Get parameters from rosparam
# # NOTE this is the Estimator, so we should use the "model" parameters. 
# # This will enable us to compare the "simulated" robot (considered the true robot location) 
# #  and the "estimated" robot (position estimated based on dead-reckoning)
#==============================================================================
wheel_width = rospy.get_param('/wheel_width_model')
body_length = rospy.get_param('/body_length')
wheel_diameter = rospy.get_param('/wheel_diameter_model')
wheel_radius = wheel_diameter/2.0


# Global variables for the robot's position
r_center_world_estimated = np.array([0.,0.])    # Position (r) of the robot in the World frame. 
theta_estimated = 0.                   # heading angle (theta) of the robot relative to the World frame. 

# Global variables for the robot's wheel displacements (to keep knowledge of it from one step to the next)
d_left_previous = 0.
d_right_previous = 0.

# Rate to set how often the estimated "pose" is published
f = 10.     # Hz 


def listener(): 
    global r_center_world_estimated, theta_estimated
    
    
# =============================================================================
#     # Launch a node called "mobile_robot_simulator"
# =============================================================================
    rospy.init_node('dead_reckoning', anonymous=False)
    
    
#==============================================================================
####    CODE HERE:
#     # Here start a Subscriber to the "/robot_wheel_displacements" topic.
#     #  If you can't remember the Message type, use "rostopic info" 
#     #  (this message is published by the "sensing_and_wheel_control_node")
#     #  Set the function "dead_reckoning" as a callback - this is the function 
#     #  that will be called when a message comes in. 
#==============================================================================
    sub_wheel_disps = rospy.Subscriber('/robot_wheel_displacements', ME439WheelDisplacements, dead_reckoning)  

    
#==============================================================================
#     # Here start a Subscriber to the "/robot_position_override" topic.
#     #  Topic "/robot_position_override", message type "Pose2D"
#     #  Call the function "set_pose" as a callback
#     #  ** Publishing on this topic elsewhere is optional, but this gives the 
#     #     node the ability to take a position override from e.g. an external 
#     #     measurement or a known starting position. 
#==============================================================================
    sub_position_override = rospy.Subscriber('/robot_position_override', Pose2D, set_pose)  
    
    
#==============================================================================
####    CODE HERE:
#     # Here a Publisher for the Estimated Robot Pose. 
#     # Topic '/robot_pose_estimated', Message type: Pose2D
#==============================================================================
    pub_robot_pose_estimated = rospy.Publisher('/robot_pose_estimated', Pose2D, queue_size = 1)
    robot_pose_estimated_message = Pose2D()

    
# =============================================================================
#     # Rate object to set a publication rate
# =============================================================================
    r = rospy.Rate(f)
    
# =============================================================================
#     # Loop to run the publication
# =============================================================================
    while not rospy.is_shutdown():
        # Publish the pose
        robot_pose_estimated_message.x = r_center_world_estimated[0]
        robot_pose_estimated_message.y = r_center_world_estimated[1]
        robot_pose_estimated_message.theta = theta_estimated
        pub_robot_pose_estimated.publish(robot_pose_estimated_message)
        
        # Log the info to the ROS log. 
        rospy.loginfo(pub_robot_pose_estimated)
        
        r.sleep()
        
# =============================================================================
# # Callback function for "dead-reckoning" (alternatively called "odometry")
# =============================================================================
def dead_reckoning(msg_in): 
    # These global variables hold this node's estimate of robot pose. 
    global r_center_world_estimated, theta_estimated
    # This parameter is necessary. 
    global wheel_width
    # More globals to store the previous values of the wheel displacements    
    global d_left_previous, d_right_previous
    
    
####    CODE HERE: extract the wheel displacements from the message in variable msg_in. 
    # REPLACE the zeros with the proper expressions. 
    # Look in the message file for ME439WheelDisplacements.msg to find the variable 
    # names for left and right wheel displacements. 
    # Or with "roscore" running, just ask ROS: "rosmsg show ME439WheelDisplacements"
    # Syntax is msg_in.variable_name
    d_left = msg_in.d_left
    d_right = msg_in.d_right
    
####    CODE HERE: Compute the CHANGE in displacement of each wheel
    # REPLACE the zeros with the proper expressions. 
    diff_left = d_left - d_left_previous
    diff_right = d_right - d_right_previous
    
####    CODE HERE: STORE the new values of d_left and d_right for the next call
    # REPLACE the zeros with the proper expressions. 
    d_left_previous = d_left
    d_right_previous = d_right
    
####    CODE HERE: compute change in path length and change in angle
    # REPLACE the zeros with the proper expressions (see lecture notes). 
    # use "diff_left" and "diff_right" which were set a few lines above. 
    diff_pathlength = (diff_left+diff_right)/2
    diff_theta = (diff_right-diff_left)/wheel_width

####    CODE HERE: compute the AVERAGE heading angle (theta) during the movement 
    # This makes the dead-reckoning more accurate than using just the old theta or the new one. 
    theta_avg = theta_estimated + diff_theta/2.
    
####    CODE HERE: compute the change in position and heading according to the dead-reckoning equations
    # REPLACE the zeros with the proper expressions (see lecture notes). 
    # Remember that sine and cosine are in the "numpy" package, which has been imported as "np"
    r_center_world_estimated[0] += diff_pathlength * -np.sin(theta_avg)     # x-direction position
    r_center_world_estimated[1] += diff_pathlength * np.cos(theta_avg)      # y-direction position
    theta_estimated += diff_theta                                           # angle

#==============================================================================
#     End of function "dead_reckoning"
#==============================================================================
    
    
    
# =============================================================================
# # Callback function for "set_pose" (to override the estimated position)
# # This node receives a "Pose2D" message type, which should be used to replace 
# # the current estimated pose. 
# =============================================================================
def set_pose(msg_in): 
    # These global variables hold this node's estimate of robot pose. 
    global r_center_world_estimated, theta_estimated
    
#    CODE HERE: extract the pose from the message file. 
    # Look in the message file for ME439WheelDisplacements.msg to find the variable names for left and right wheel displacements. 
    # Or with "roscore" running, just ask ROS: "rosmsg show ME439WheelDisplacements"
    r_center_world_estimated[0] = msg_in.x
    r_center_world_estimated[1] = msg_in.y
    theta_estimated = msg_in.theta
        
#==============================================================================
#     # End of function "set_pose"
#==============================================================================
    
    
if __name__ == '__main__':
    try: 
        listener()
    except rospy.ROSInterruptException: 
        pass
#        traceback.print_exc()
