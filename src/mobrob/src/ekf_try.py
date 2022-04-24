#!/usr/bin/env python3

# =============================================================================
# Peter G. Adamczyk, Mohamed Safwat, Ahmed Khalil
# Updated 2022-04-09
# =============================================================================


from math import expm1
from mobrob_util import msg
import rospy
import numpy as np
import traceback 
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32
import time
from mobrob_util.msg import ME439SensorsProcessed,ME439WheelSpeeds, ME439WheelDisplacements, IMU

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



# 
# Global variables for the robot's position
r_center_world_estimated = np.array([0.,0.])    # Position (r) of the robot in the World frame. 
theta_estimated = 0.                   # heading angle (theta) of the robot relative to the World frame. 

# Global variables for the robot's wheel displacements (to keep knowledge of it from one step to the next)
d_left_previous = 0.
d_right_previous = 0.



# Rate to set how often the estimated "pose" is published
f = 10.     # Hz 
class Ekf:
    def __init__ (self):

        self.P_k_1 = np.eye(3)     #initialize covariance matrix
        self.Q = 0.02*np.eye(3)  #process noise covariance 
        
        self.gyrodT =1/200



        self.A = np.eye(3)          

        self.R = 0.4*np.eye(3)  #measurement noise covariance
        self.e0 = 0
        self.e0_prev = 0
        self.e1 = 0
        self.e1_prev =0
        self.gyro = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        self.v_r = 0
        self.v_l = 0
        self.v_r_in = 0
        self.v_l_in = 0
        self.e0 = 0
        self.e1 = 0
        self.gyro = 0
        self.dt = 1/f
        #x_hat is x,y,theta, x_dot, y_dot
        self.x_hat_k_1 = np.array([[0, 0, 0]]).T     #estimated state column vector
        self.gyroTprev = 0
        self.encTprev = 0
        self.counterG = 0
        self.counterE = 0
        self.encdT = 0.01

            # =============================================================================
    #     # Launch a node called "mobile_robot_simulator"
    # =============================================================================
        rospy.init_node('ekf', anonymous=False)
        
    # =============================================================================
    #     # Listen to the Encoder and gyro data"
    # =============================================================================    

        sub_gyro = rospy.Subscriber('/yaw_gyro', IMU, self.gyro_listener)

        #sub_enc_vel = rospy.Subscriber('/robot_wheel_vel',  ME439WheelSpeeds, self.enc_vel) 

        sub_enc_disp = rospy.Subscriber('/robot_wheel_displacements', ME439WheelDisplacements, self.enc_disp)

    # =============================================================================
    #     # Listen to the control inputs"
    # =============================================================================        

        sub_vel_input = rospy.Subscriber('/wheel_speeds_desired', ME439WheelSpeeds, self.vel_input) 

    def gyro_listener(self, msg_in):
        if self.counterG > 0:
            self.gyroTnow = float(msg_in.stamp.secs + msg_in.stamp.nsecs/(10**9))
            self.gyrodT = self.gyroTnow - self.gyroTprev
            self.gyro += msg_in.yaw*self.gyrodT#*np.pi/180.0
            self.gyroTprev = self.gyroTnow
        else:
            self.gyroTprev = float(time.time())
            self.counterG+=1

    def enc_disp(self, msg_in):
        if self.counterE > 0:
            self.encTnow = float(msg_in.stamp.secs + msg_in.stamp.nsecs/(10**9))
            self.e0_now = msg_in.d_left
            self.e1_now = msg_in.d_right
        
            self.encdT = self.encTnow-self.encTprev

            self.e0 = (self.e0_now - self.e0_prev)/self.encdT
            self.e1 = (self.e1_now - self.e1_prev)/self.encdT
            self.e0_prev = self.e0_now
            self.e1_prev = self.e1_now
            self.encTprev = self.encTnow
        else:
            self.encTprev = float(time.time())
            self.counterE+=1

    # def enc_vel(self,msg_in):
  
    #     self.e0_now = msg_in.v_left
    #     self.e1_now = msg_in.v_right
    #     self.e0 = self.e0_now - self.e0_prev
    #     self.e1 = self.e1_now - self.e1_prev
    #     self.e0_prev = self.e0_now
    #     self.e1_prev = self.e1_now



    def vel_input(self,msg_in):
        
        self.v_r_in = msg_in.v_right
        self.v_l_in = msg_in.v_left

    # # =============================================================================
    # # # Callback function for "set_pose" (to override the estimated position)
    # # # This node receives a "Pose2D" message type, which should be used to replace 
    # # # the current estimated pose. 
    # # =============================================================================
    # def set_pose(msg_in): 
    #     # These global variables hold this node's estimate of robot pose. 
    #     global r_center_world_estimated, theta_estimated
        
    # #    CODE HERE: extract the pose from the message file. 
    #     # Look in the message file for ME439WheelDisplacements.msg to find the variable names for left and right wheel displacements. 
    #     # Or with "roscore" running, just ask ROS: "rosmsg show ME439WheelDisplacements"
    #     r_center_world_estimated[0] = msg_in.x
    #     r_center_world_estimated[1] = msg_in.y
    #     theta_estimated = msg_in.theta
            
    # #==============================================================================
    # #     # End of function "set_pose"
    # #==============================================================================
    
    
if __name__ == '__main__':
    try:
        ek = Ekf()

    


            #==============================================================================
        ####    CODE HERE:
        #     # Here a Publisher for the Estimated Robot Pose. 
        #     # Topic '/robot_pose_estimated', Message type: Pose2D
        #==============================================================================
        pub_robot_pose_estimated = rospy.Publisher('/robot_pose_ekf', Pose2D, queue_size = 1)
        robot_pose_estimated_message = Pose2D()

        # =============================================================================
        #     # Rate object to set a publication rate
        # =============================================================================
        r = rospy.Rate(f)
        while not rospy.is_shutdown():
            z_k_1 = np.array([[ek.e1,ek.e0,ek.gyro]]).T 
            
            u_k_1 = np.array([[ek.v_r_in,ek.v_l_in]]).T     #control inputs
            # v_r = ek.x_hat_k_1[3][0]
            # v_l = ek.x_hat_k_1[4][0]
            theta = ek.x_hat_k_1[2][0]
            
            B = np.array([      #input matrix
            [0.5*np.cos(theta)*ek.dt   ,0.5*np.cos(theta)*ek.dt],
            [0.5*np.sin(theta)*ek.dt   ,0.5*np.cos(theta)*ek.dt],
            [ek.dt/wheel_width         ,-ek.dt/wheel_width     ],
            ])   
            # Publish the pose
            x_hat_k = ek.A@ek.x_hat_k_1 +  B@u_k_1
            #print(x_hat_k)
            P_k = ek.A@ek.P_k_1@ek.A.T +ek.Q

            H = np.linalg.inv(np.array([      #input matrix
            [0.5*np.cos(theta)*ek.dt   ,0.5*np.cos(theta)*ek.dt, 0],
            [0.5*np.sin(theta)*ek.dt   ,0.5*np.cos(theta)*ek.dt, 0],
            [0                         ,0                      , 1],
            ]))     #state to measurement matrix        
            #print(P_k)
            K = P_k@H.T@np.linalg.inv(H@P_k@H.T + ek.R)
            x_k_new = x_hat_k + K@(z_k_1 - H@x_hat_k)
            P_k_new = (np.identity(3) - K@H)@P_k #P_k - K@ek.H@P_k
     
            robot_pose_estimated_message.x = x_k_new[0][0]
            robot_pose_estimated_message.y = x_k_new[1][0]
            robot_pose_estimated_message.theta = x_k_new[2][0]
            
            pub_robot_pose_estimated.publish(robot_pose_estimated_message)
            # Log the info to the ROS log. 
            #rospy.loginfo(x_k_new)
            ek.x_hat_k_1 = x_k_new
            ek.P_k_1 = P_k_new
            r.sleep()
        rospy.spin()

    except rospy.ROSInterruptException: 
        pass
#        traceback.print_exc()
