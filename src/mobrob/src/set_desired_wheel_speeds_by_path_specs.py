#!/usr/bin/env python3

# =============================================================================
# Peter G. Adamczyk 
# 2018-10
# Updated 2021-02-26
# =============================================================================

import rospy
import traceback 
import numpy as np
# IMPORT the custom message: 
# we import it "from" the ROS package we created it in (here "mobrob_util") with an extension of .msg ...
# and actually import the message type by name (here "ME439WheelSpeeds")
from mobrob_util.msg import ME439WheelSpeeds


# =============================================================================
#     Set up a time course of commands
# =============================================================================

# =============================================================================
# # NEW: Determine paths by lines, arcs, pivots and pauses, and create a 
# #  "robot" object to plan it for you. 
# =============================================================================

# Get parameters from rosparam
wheel_width = rospy.get_param('/wheel_width_model') # All you have when planning is a model - you never quite know the truth! 
body_length = rospy.get_param('/body_length')
wheel_diameter = rospy.get_param('/wheel_diameter_model')
wheel_radius = wheel_diameter/2.0

####    CODE HERE:
# Create a mobile robot object from the Imported module "me439_mobile_robot_class"
# REMEMBER to call the right file (i.e., use the _HWK if needed)
import me439_mobile_robot_class_v01 as m439rbt
robot = m439rbt.robot(wheel_width, body_length, wheel_radius)

####    CODE HERE:
# Specify stage_settings as a collections of lines, arcs, pivots and pauses
# Use the functions in your Class (imported above as "me439rbt")
# Example: Move Forward and Back, 0.3 meters per second:  
# ** NOTE the signs on the backward driving: backward speed and backward distance! 
#stage_settings = np.array( [ robot.plan_pause(1.0), robot.plan_line(0.3, 0.2), robot.plan_line(-0.1, -0.2), robot.plan_pause(2.0)] )
# Example: pause, forward, pause, pivot right 180 deg, pause, return to home, pause, turn, pause. 
# ** NOTE the signs on the Omega and Angle for the "plan_pivot" calls! 
stage_settings = np.array( [ robot.plan_pause(1.0), robot.plan_line(0.1, 0.3), robot.plan_pause(1.0), robot.plan_pivot(-1.0, -np.pi), robot.plan_pause(1.0), robot.plan_line(0.1, 0.3), robot.plan_pause(1.0), robot.plan_pivot(1.0, np.pi), robot.plan_pause(1.0)] )
####    CODE HERE: ADD YOUR OWN  


# Convert it into a numpy array
stage_settings_array = np.array(stage_settings)
# Convert the first column to a series of times (elapsed from the beginning) at which to switch settings. 
stage_settings_array[:,0] = np.cumsum(stage_settings_array[:,0],0)  # cumsum = "cumulative sum". The last Zero indicates that it should be summed along the first dimension (down a column). 

# =============================================================================
# # END of new section on planning with lines, arcs, pivots and pauses
# =============================================================================


# Publish desired wheel speeds at the appropriate time. 
def talker(): 
    # Actually launch a node called "set_desired_wheel_speeds_by_path_specs"
    rospy.init_node('set_desired_wheel_speeds_by_path_specs', anonymous=False)

    # Create the publisher. Name the topic "sensors_data", with message type "Sensors"
    pub_speeds = rospy.Publisher('/wheel_speeds_desired', ME439WheelSpeeds, queue_size=10)
    # Declare the message that will go on that topic. 
    # Here we use one of the message name types we Imported, and add parentheses to call it as a function. 
    # We could also put data in it right away using . 
    msg_out = ME439WheelSpeeds()
    msg_out.v_left = 0
    msg_out.v_right = 0
    
    # set up a rate basis to keep it on schedule.
    r = rospy.Rate(100) # N Hz
    try: 
        # start a loop 
        t_start = rospy.get_rostime()
        
        while not rospy.is_shutdown():
            future_stages = np.argwhere( stage_settings_array[:,0] >= (rospy.get_rostime()-t_start).to_sec() ) 
            if len(future_stages)>0:
                stage = future_stages[0]
                print(stage)
            else: 
                break
            msg_out.v_left = stage_settings_array[stage,1]
            msg_out.v_right = stage_settings_array[stage,2]
            # Actually publish the message
            pub_speeds.publish(msg_out)
            # Log the info (optional)
#            rospy.loginfo(pub_speeds)    
            
            r.sleep()
        
#        # Here step through the settings. 
#        for stage in range(0,len(stage_settings_array)):  # len gets the length of the array (here the number of rows)
#            # Set the desired speeds
#            dur = stage_settings_array[stage,0]
#            msg_out.v_left = stage_settings_array[stage,1]
#            msg_out.v_right = stage_settings_array[stage,2]
#            # Actually publish the message
#            pub_speeds.publish(msg_out)
#            # Log the info (optional)
#            rospy.loginfo(pub_speeds)       
#            
#            # Sleep for just long enough to reach the next command. 
#            sleep_dur = dur - (rospy.get_rostime()-t_start).to_sec()
#            sleep_dur = max(sleep_dur, 0.)  # In case there was a setting with zero duration, we will get a small negative time. Don't use negative time. 
#            rospy.sleep(sleep_dur)
        
    except Exception:
        traceback.print_exc()
        # When done or Errored, Zero the speeds
        msg_out.v_left = 0
        msg_out.v_right = 0
        pub_speeds.publish(msg_out)
        rospy.loginfo(pub_speeds)    
        pass
        
        
    # When done or Errored, Zero the speeds
    msg_out.v_left = 0
    msg_out.v_right = 0
    pub_speeds.publish(msg_out)
    rospy.loginfo(pub_speeds)    


if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        pass
