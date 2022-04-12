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
# we import it "from" the ROS package we created it in (here "me439robot") with an extension of .msg ...
# and actually import the message type by name (here "ME439SensorsRaw")
from mobrob_util.msg import ME439SensorsRaw
from mobrob_util.msg import ME439SensorsProcessed

# Initialize globals
t_previous = 0. 
e0_previous = 0 
e1_previous = 0
e0_offset = 0
e1_offset = 0
            
counts_per_encoder_revolution = rospy.get_param('/counts_per_encoder_revolution')  # 12.0
gear_ratio = rospy.get_param('/gear_ratio') # 75.81262327416174  # standard ME439 robot: gear_ratio = 75.81262327416174 = (34.*34.*35.*38.)/(13.*12.*13.*10.)   
encoder_counts_to_radians = 1./counts_per_encoder_revolution/gear_ratio*2.0*np.pi
wheel_radius = rospy.get_param('/wheel_diameter_actual')/2.0 # 0.030 # meters
speed_of_sound_meterspersec = rospy.get_param('/speed_of_sound_meterspersec')

e0_direction_sign = rospy.get_param('/left_encoder_sign')
e1_direction_sign = rospy.get_param('/right_encoder_sign')



initializing = 1

pub = rospy.Publisher('/sensors_data_processed', ME439SensorsProcessed, queue_size=10)

def listener(): 
    rospy.init_node('sensors_processing_node', anonymous=False)
    sub = rospy.Subscriber('/sensors_data_raw', ME439SensorsRaw, sensors_process) # Subscribe to the "sensors_data_raw" topic
   
    rospy.spin()    # keep the node from exiting
    

def sensors_process(msg_in):
    # bring the Globals into this function's scope
    global t_previous, e0_previous, e1_previous, e0_offset, e1_offset, encoder_counts_to_radians, initializing, pub

    try:     
        if initializing:
            t_previous = msg_in.t
            e0_offset = msg_in.e0
            e1_offset = msg_in.e1
            e0_previous = msg_in.e0 - e0_offset
            e1_previous = msg_in.e1 - e1_offset
            initializing = 0
        else: 
            msg_out = ME439SensorsProcessed()
            
            msg_out.dt = (msg_in.t - t_previous)
            dt = msg_out.dt.to_sec()
            
            e0_current = (msg_in.e0 - e0_offset) * e0_direction_sign
            e1_current = (msg_in.e1 - e1_offset) * e1_direction_sign
            
            msg_out.e0radians = e0_current * encoder_counts_to_radians 
            msg_out.e1radians = e1_current * encoder_counts_to_radians 
            msg_out.e0meters = msg_out.e0radians*wheel_radius
            msg_out.e1meters = msg_out.e1radians*wheel_radius
            msg_out.e0radpersec = (e0_current - e0_previous)/(dt) * encoder_counts_to_radians
            msg_out.e1radpersec = (e1_current - e1_previous)/(dt) * encoder_counts_to_radians
            msg_out.e0meterspersec = msg_out.e0radpersec * wheel_radius 
            msg_out.e1meterspersec = msg_out.e1radpersec * wheel_radius 
            
            msg_out.a0 = float(msg_in.a0)
            msg_out.a1 = float(msg_in.a1)
            msg_out.a2 = float(msg_in.a2)
            msg_out.a3 = float(msg_in.a3)
            msg_out.a4 = float(msg_in.a4)
            msg_out.a5 = float(msg_in.a5)
            
            msg_out.u0meters = msg_in.u0/10.0e6 * speed_of_sound_meterspersec
            msg_out.u1meters = msg_in.u1/10.0e6 * speed_of_sound_meterspersec
            msg_out.u2meters = msg_in.u2/10.0e6 * speed_of_sound_meterspersec
            
            # Update "previous" values
            t_previous = msg_in.t
            e0_previous = e0_current
            e1_previous = e1_current            
            
            # Publish the Processed message
            pub.publish(msg_out)
            # Log the info (optional)
            rospy.loginfo(pub)       
            
    except Exception:
        traceback.print_exc()
        pass        



if __name__ == '__main__':
    listener()
#    try: 
#        listener()
#    except rospy.ROSInterruptException: 
#        pass
