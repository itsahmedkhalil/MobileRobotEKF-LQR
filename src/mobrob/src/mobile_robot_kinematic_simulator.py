#!/usr/bin/env python3

# =============================================================================
# Peter G. Adamczyk 
# 2018-10-11
# Updated 2021-02-26
# =============================================================================


import rospy
import traceback 
import me439_mobile_robot_class_v00 as m439rbt
from geometry_msgs.msg import Pose2D
from mobrob_util.msg import ME439WheelSpeeds, ME439WheelAngles, ME439WheelDisplacements

#==============================================================================
# # Get parameters from rosparam
#==============================================================================
wheel_width = rospy.get_param('/wheel_width_actual')
body_length = rospy.get_param('/body_length')
wheel_diameter = rospy.get_param('/wheel_diameter_actual')
wheel_radius = wheel_diameter/2.0


t_previous = None
f = 100.    # Simulation rate of the simulator node
n = 10     # Publication rate of the Pose

def simulate(): 
    global t_previous, f, n
    
    
# =============================================================================
#     # Launch a node called "mobile_robot_simulator"
# =============================================================================
    rospy.init_node('mobile_robot_simulator', anonymous=False)
    t_previous = rospy.get_rostime()
# =============================================================================
#     #Create a mobile robot object from the Imported module "me439_mobile_robot_class"
# =============================================================================
    robot = m439rbt.robot(wheel_width, body_length, wheel_radius)
    
#==============================================================================
#     # Here start a Subscriber to the "wheel_speeds_desired" topic.
#==============================================================================
    #   NOTE the Callback to the set_wheel_speeds function of the robot class.
    #   This will update  
    #   NOTE also the extra arguments to that callback: the Motor Encoders (both in a list)
    sub_wheel_speeds = rospy.Subscriber('/wheel_speeds_desired', ME439WheelSpeeds, set_wheel_speed_targets,robot)  
    
    pub_robot_pose = rospy.Publisher('/robot_pose_simulated', Pose2D, queue_size = 1)
    robot_pose_message = Pose2D()
    pub_robot_wheel_angles = rospy.Publisher('/robot_wheel_angles_simulated', ME439WheelAngles, queue_size = 10)
    robot_wheel_angles_message = ME439WheelAngles()
    pub_robot_wheel_displacements = rospy.Publisher('/robot_wheel_displacements_simulated', ME439WheelDisplacements, queue_size = 10)
    robot_wheel_displacements_message = ME439WheelDisplacements()
    
    # Rate object to set a simulation rate
    r = rospy.Rate(f)
    
# =============================================================================
#     # Loop to run the simulation
# =============================================================================
    while not rospy.is_shutdown():
        # Count to n here to prevent publishing too often
        for ii in range(n): 
            t_current = rospy.get_rostime()
            dt = (t_current - t_previous).to_sec()
            t_previous = t_current          # save the current time as the previous time, for the next use. 
            robot.integration_step(dt)
            
            
            r.sleep()    # keep this node from exiting
        
# =============================================================================
#         # when it gets here (every n-th simulation step) we want to actually publish the data
# =============================================================================
#        # Maybe log the current position?
#        robot.append_current_position_to_history()
        
        # Now publish the pose
        robot_pose_message.x = robot.r_center_world[0]
        robot_pose_message.y = robot.r_center_world[1]
        robot_pose_message.theta = robot.theta
        pub_robot_pose.publish(robot_pose_message)
        
        # And the encoder angles
        robot_wheel_angles_message.ang_left = robot.left_wheel_angle
        robot_wheel_angles_message.ang_right = robot.right_wheel_angle
        pub_robot_wheel_angles.publish(robot_wheel_angles_message)
        
        # And the wheel displacements
        robot_wheel_displacements_message.d_left = robot.left_wheel_distance_traveled
        robot_wheel_displacements_message.d_right = robot.right_wheel_distance_traveled
        pub_robot_wheel_displacements.publish(robot_wheel_displacements_message)
        
        rospy.loginfo(pub_robot_pose)
        
        
# =============================================================================
# # Callback function to set wheel speeds in the robot object
# =============================================================================
def set_wheel_speed_targets(msg_in, robot): 
    global t_previous
    t_current = rospy.get_rostime()
    dt = (t_current - t_previous).to_sec()
    t_previous = t_current      # save the current time as the previous time, for the next use. 
    robot.integration_step(dt)
    robot.set_wheel_speeds(msg_in.v_left, msg_in.v_right)
    
    
    
    
    
    
if __name__ == '__main__':
    try: 
        simulate()
    except rospy.ROSInterruptException: 
        traceback.print_exc()
