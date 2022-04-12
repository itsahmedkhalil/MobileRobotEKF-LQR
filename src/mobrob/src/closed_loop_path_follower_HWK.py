#!/usr/bin/env python3

# =============================================================================
# Peter G. Adamczyk 
# 2018-10
# Updated 2021-02-27
# =============================================================================

import rospy
import traceback 
import numpy as np
# IMPORT the custom message: 
# we import it "from" the ROS package we created it in (here "mobrob_util") with an extension of .msg ...
# and actually import the message types by name (
from mobrob_util.msg import ME439WheelSpeeds, ME439PathSpecs #, ME439PathSegComplete
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import me439_mobile_robot_class_v02 as m439rbt  # REMEMBER to call the right file (version, or use the _HWK if needed)

# global variable to hold the path specs currently being tracked
path_segment_spec = ME439PathSpecs()
path_segment_spec.x0 = 0.
path_segment_spec.y0 = 0.
path_segment_spec.theta0 = 0.
path_segment_spec.Radius = np.inf
path_segment_spec.Length = np.inf
    
# global variables used in path following: 
initialize_psi_world= True
estimated_pose_previous = Pose2D()
estimated_x_local_previous = 0.
estimated_theta_local_previous= 0. 
path_segment_curvature_previous = 0.
estimated_psi_world_previous = 0. 
estimated_segment_completion_fraction_previous = 0.


# =============================================================================
#     Set up a closed-loop path controller
#     Subscribe to "robot_pose_estimated" (Pose2D) 
#     and Publish "wheel_speeds_desired" (ME439WheelSpeeds)
# =============================================================================

# Get parameters from rosparam
wheel_width = rospy.get_param('/wheel_width_model') # All you have when planning is a model - you never quite know the truth! 
body_length = rospy.get_param('/body_length')
wheel_diameter = rospy.get_param('/wheel_diameter_model')
wheel_radius = wheel_diameter/2.0

####    CODE HERE: 
# Closed-loop controller parameters: get them from the ROS parameters imported from the YAML file. 
Vmax = 0.0 
Beta = 0.0 
gamma = 0.0 
angle_focus_factor = 0.0 
####    CODE END

# Create a mobile robot object from the Imported module "me439_mobile_robot_class"
robot = m439rbt.robot(wheel_width, body_length, wheel_radius)

# initial situation is that the path is NOT complete. 
path_is_complete = False

####    CODE HERE: Create two publishers as described
# Create the publisher. Name the topic "wheel_speeds_desired", with message type "ME439WheelSpeeds"
pub_speeds = []

# Create the publisher for "segment_complete". Name the topic "segment_complete", with message type "Bool"
pub_segment_complete = []
####    CODE END

# Set up the node and its subscriptions, and keep it alive.
def start_node(): 
    # Actually launch a node called "closed_loop_path_follower"
    rospy.init_node('closed_loop_path_follower', anonymous=False)
    
    # Create a Subscriber to the robot's current estimated position
    # with a callback to "path_follow"
    sub_robot_pose_estimated = rospy.Subscriber('/robot_pose_estimated', Pose2D, path_follow)
    
    # Create a Subscriber to the path the robot is trying to follow
    # With a callback to "update_path"
    sub_path_segment_spec = rospy.Subscriber('/path_segment_spec', ME439PathSpecs, update_path)

    # Subscriber to the "path_complete" topic
    sub_path_complete = rospy.Subscriber('/path_complete', Bool, set_path_complete)
        
    # Prevent the node from exiting
    rospy.spin()    
    
    # When done or Errored, Zero the speeds
    global pub_speeds
    # Set up the message that will go on that topic. 
    msg_out = ME439WheelSpeeds()
    msg_out.v_left = 0
    msg_out.v_right = 0
    pub_speeds.publish(msg_out)
    rospy.loginfo(pub_speeds)    

# =============================================================================
# # Function to tell the path segment specifier node whether the path is finished
# =============================================================================
def set_path_complete(msg_in):
    global path_is_complete
    path_is_complete = msg_in.data

# =============================================================================
# # Function to update the path segment currently being tracked
# =============================================================================
def update_path(path_msg_in):
    global path_segment_spec
    path_segment_spec = path_msg_in
    
# =============================================================================
# # Function to update the path tracking control based on the robot's estimated position
# =============================================================================
def path_follow(pose_msg_in):
    # First assign the incoming message
    global estimated_pose, path_segment_spec
    estimated_pose = pose_msg_in
    pose_xy = np.array([estimated_pose.x, estimated_pose.y])
    pose_theta = np.array([estimated_pose.theta])
    if np.isinf(path_segment_spec.Length):
        return; 
    # Set up global variables
    global initialize_psi_world, estimated_pose_previous, estimated_x_local_previous, estimated_theta_local_previous, path_segment_curvature_previous, estimated_psi_world_previous, estimated_segment_completion_fraction_previous
    
    # Create a vector variable for the Origin of the path segment 
    path_segment_Origin = np.array([path_segment_spec.x0, path_segment_spec.y0] )

    # Forward distance relative to a Line path is computed along the Forward axis. 
    # Therefore Define the Forward Axis: 
    path_segment_y0_vector = np.array([-np.sin(path_segment_spec.theta0), np.cos(path_segment_spec.theta0)])
    # local X is computed perpendicular to the segment. 
    # Therefore define the Perpendicular axis.
    path_segment_x0_vector = np.array([-np.sin(path_segment_spec.theta0 - np.pi/2.0), np.cos(path_segment_spec.theta0 - np.pi/2.0)])
    # Define path curvature
    path_segment_curvature = 1.0/path_segment_spec.Radius
    
    
# =============================================================================
#     ### First Compute the robot's position relative to the path (x, y, theta)
#     ### and the local path properties (curvature 1/R and segment completion percentage)
# =============================================================================
        
    # If it is a Line (Infinite radius)
    if np.isinf(path_segment_spec.Radius):
        path_segment_endpoint = path_segment_Origin + path_segment_spec.Length*path_segment_y0_vector
        
        # compute position relative to Segment: 
        estimated_xy_rel_to_segment_origin = (pose_xy - path_segment_Origin)   # XY vector from Origin of segment to current location of robot. 
        # Projection of vector from path origin to current position 
        estimated_segment_forward_pos = estimated_xy_rel_to_segment_origin.dot(path_segment_y0_vector)
        # The fraction completion can be estimated as the path length the robot has gone through, as a fraction of total path length on this segment
        estimated_segment_completion_fraction = estimated_segment_forward_pos / path_segment_spec.Length
        # Position of the robot to the Right of the segment Origin
        estimated_segment_rightward_pos = estimated_xy_rel_to_segment_origin.dot(path_segment_x0_vector)
        
        estimated_y_local = 0.0   # y_local = 0 by definition: Local coords are defined relative to the closest point on the path. 
        estimated_x_local = estimated_segment_rightward_pos
        estimated_theta_local = pose_theta - path_segment_spec.theta0 
    
    # Arc
    else:   
        curve_sign = np.sign(path_segment_spec.Radius)
        path_segment_circle_center = path_segment_Origin + path_segment_spec.Radius*-path_segment_x0_vector
        # determine the angular displacement of this arc. SIGNED quantity! 
        path_segment_angular_displacement = path_segment_spec.Length/path_segment_spec.Radius
        path_segment_ThetaEnd = path_segment_spec.theta0 + path_segment_angular_displacement
        estimated_xy_rel_to_circle_center = (pose_xy - path_segment_circle_center)
        
        # Compute angle of a vector from circle center to Robot, in the world frame, relative to the +Yworld axis. 
        # Note how this definition affects the signs of the arguments to "arctan2"
        estimated_psi_world = np.arctan2(-estimated_xy_rel_to_circle_center[0], estimated_xy_rel_to_circle_center[1])
        # unwrap the angular displacement
        if initialize_psi_world:
            estimated_psi_world_previous = estimated_psi_world
            initialize_psi_world = False
        while estimated_psi_world - estimated_psi_world_previous > np.pi: # was negative, is now positive --> should be more negative. 
            estimated_psi_world += -2.0*np.pi
        while estimated_psi_world - estimated_psi_world_previous < -np.pi: # was positive and is now negative --> should be more positive. 
            estimated_psi_world += 2.0*np.pi
        
        # update the "previous angle" memory. 
        estimated_psi_world_previous = estimated_psi_world
        # The local path forward direction is perpendicular (clockwise) to this World frame origin-to-robot angle. 
        estimated_path_theta = estimated_psi_world + np.pi/2.0*curve_sign
        # The fraction completion can be estimated as the path angle the robot has gone through, as a fraction of total angular displacement on this segment
        estimated_segment_completion_fraction = (estimated_path_theta - path_segment_spec.theta0) / path_segment_angular_displacement

        estimated_y_local = 0.0  # by definition of local coords
        # x_local is positive Inside the circle for Right turns, and Outside the circle for Left turns
        estimated_x_local = curve_sign*(np.sqrt(np.sum(np.square(estimated_xy_rel_to_circle_center))) - np.abs(path_segment_spec.Radius) ) 
        estimated_theta_local = pose_theta - estimated_path_theta
    
    ## Whether Line or Arc, update the "local" coordinate state and path properties: 
    estimated_theta_local = m439rbt.fix_angle_pi_to_neg_pi(estimated_theta_local)

    # Update the "previous" values 
    estimated_pose_previous = estimated_pose
    estimated_x_local_previous = estimated_x_local
    estimated_theta_local_previous = estimated_theta_local
    path_segment_curvature_previous = path_segment_curvature
    estimated_segment_completion_fraction_previous = estimated_segment_completion_fraction    


# =============================================================================
#     ### CONTROLLER for path tracking based on local position and curvature. 
#     # parameters for the controller are 
#     #   Vmax: Maximum allowable speed,  
#     # and controller gains:
#     #   Beta (gain on lateral error, mapping to lateral speed)
#     #   gamma (gain on heading error, mapping to rotational speed)
#     # and a control variable for the precision of turns, 
#     #   angle_focus_factor
# =============================================================================
    global Vmax, Beta, gamma, angle_focus_factor
    
####    CODE HERE:  Put in formulas for anything that is 0.0, and try the "TRY THIS" variations. 
    # First set the speed with which we want the robot to approach the path
    xdot_local_desired = 0.0   # Use formula from Lecture
    # limit it to +-Vmax
    xdot_local_desired = np.min([np.abs(xdot_local_desired),abs(Vmax)])*np.sign(xdot_local_desired)
    
    # Next set the desired theta_local 
    theta_local_desired = 0.0   # Use formula from Lecture
            
    ## Next SET SPEED OF ROBOT CENTER. 
    ## G. Cook 2011 says just use constant speed all the time,
    ## TRY THIS FIRST
    Vc = Vmax    

    ## But, that drives farther from the path at first if it is facing away. 
    ## This FIX causes the speed to fall to zero if the robot is more than 90 deg from the heading we want it to have. 
    ##   The parameter "angle_focus_factor" can make it even more restrictive if needed (e.g. angle_focus_factor = 2 --> 45 deg limit). 
    ##   Value of 1.0 uses a straight cosine of the angle. 
    ## TRY WITH AND WITHOUT THIS 
    Vc = Vmax * np.cos(angle_focus_factor * m439rbt.fix_angle_pi_to_neg_pi(theta_local_desired - estimated_theta_local))
    
    ## Could also limit it to only forward. 
    ## TRY WITH AND WITHOUT THIS 
    Vc = np.max([Vc,0])
    
    # Finally set the desired angular rate
    estimated_theta_local_error = m439rbt.fix_angle_pi_to_neg_pi(theta_local_desired - estimated_theta_local)
    omega = 0.0
####    CODE END    
    
    # Finally, use the "robot" object created elsewhere (member of the me439_mobile_robot_xx class) to translate omega and Vc into wheel speeds
    global robot
    robot.set_wheel_speeds_from_robot_velocities(Vc, omega)
    
    # Check if the overall path is complete 
    # If so, stop!
    global path_is_complete    
    if path_is_complete:
        robot.set_wheel_speeds(0.,0.)    
    
    # Now Publish the desired wheel speeds
    global pub_speeds
    # Set up the message that will go on that topic. 
    msg_speeds = ME439WheelSpeeds()
    msg_speeds.v_left = robot.left_wheel_speed
    msg_speeds.v_right = robot.right_wheel_speed
    pub_speeds.publish(msg_speeds)
    
    print(estimated_segment_completion_fraction)    
    
    # Finally, if the segment is complete, publish that fact
    if estimated_segment_completion_fraction >= 1.0:
        global pub_segment_complete
        msg_seg_complete = Bool()
        msg_seg_complete.data = True
        pub_segment_complete.publish(msg_seg_complete)
        
        # and clear the memory of angle wrapping:
        initialize_psi_world = True
    

if __name__ == '__main__':
    try: 
        start_node()
    except rospy.ROSInterruptException: 
        pass
