#!/usr/bin/env python3

# =============================================================================
# Peter G. Adamczyk 
# 2018-10-11
# Updated 2021-10-19
# =============================================================================

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np 
import rospy
import traceback 
import me439_mobile_robot_class_v00 as m439rbt
from geometry_msgs.msg import Pose2D

#==============================================================================
# # Get parameters from rosparam
#==============================================================================
wheel_width = rospy.get_param('/wheel_width_actual')
body_length = rospy.get_param('/body_length')
wheel_diameter = rospy.get_param('/wheel_diameter_actual')
wheel_radius = wheel_diameter/2.0


def listener(): 
    # =============================================================================
    #     # Launch a node called "mobile_robot_animator"
    # =============================================================================
    rospy.init_node('mobile_robot_animator', anonymous=False)
        
    # =============================================================================
    #     #Create a mobile robot object from the Imported module "me439_mobile_robot_class"
    # =============================================================================
    robot = m439rbt.robot(wheel_width, body_length, wheel_radius)
    
    sub_robot_pose = rospy.Subscriber('/robot_pose_simulated', Pose2D, set_robot_pose, robot)
    
    # Call the animation function. This has a loop in it, so it won't exit.
    animate_robot(robot)
    
    
def set_robot_pose(msg_in, robot):
    robot.r_center_world = np.array([msg_in.x, msg_in.y])
    robot.theta= msg_in.theta
    robot.append_current_position_to_history()
    
    
def animate_robot(robot):
    # Set up the Animation plot
    fig1= plt.figure()
    
    robotoutline, = plt.plot([], [], 'r-')
    robotpath, = plt.plot([],[], 'b--')
    plt.axis('equal')   # Note for some reason this has to come before the "xlim" and "ylim" or an explicit axis limits command "plt.axis([xmin,xmax,ymin,ymax])"
    plt.xlim(-1, 1)
    plt.ylim(-1, 1)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title(rospy.get_name())  # Title the plot with the name of the node, obtained from ROS.
    
    dt = 0.5   # seconds
    
    line_ani = animation.FuncAnimation(fig1, update_drawing, frames=1, fargs=(robot, robotoutline, robotpath), interval= (dt*1000), repeat=True, blit=False)
    plt.show()
    return line_ani

def update_drawing(num, robot, robotoutline, robotpath):  # "num" is the current frame number - doesn't do anything in this case
    current_outline = robot.get_outline_world_xy()
    past_path = robot.position_history_world   
    
    robotoutline.set_data(current_outline[0,:],current_outline[1,:])
    robotpath.set_data(past_path[:,0], past_path[:,1])
    
    ## Find limits of the plot and expand them to make sure the robot is on screen
    axlims = plt.axis()
    xmin = axlims[0]
    xmax = axlims[1]
    xwidth = xmax-xmin
    ymin = axlims[2]
    ymax = axlims[3]
    ywidth = ymax-ymin
    current_position = robot.r_center_world
    # Check robot position and expand the frame if necessary
    if (current_position[0] - xmin)/xwidth < 0.1:
        xmin = xmax - 1.5*xwidth
    if (current_position[0] - xmin)/xwidth > 0.9:
        xmax = xmin + 1.5*xwidth
    if (current_position[1] - ymin)/ywidth < 0.1:
        ymin = ymax - 1.5*ywidth
    if (current_position[1] - ymin)/ywidth > 0.9:
        ymax = ymin + 1.5*ywidth
    plt.axis([xmin,xmax,ymin,ymax])
    
    return robotoutline, robotpath
    
if __name__ == '__main__':
    try: 
        listener()
    except rospy.ROSInterruptException: 
        traceback.print_exc()
