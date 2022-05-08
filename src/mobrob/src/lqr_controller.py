#!/usr/bin/env python3

# Author: Ahmed Khalil, Mohamed Safwat
# Adapted from: https://automaticaddison.com and https://github.com/AtsushiSakai/PythonRobotics
# Automatic Addison: https://automaticaddison.com/linear-quadratic-regulator-lqr-with-python-code-example/
# Python Robotics: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/lqr_speed_steer_control/lqr_speed_steer_control.py
 
import numpy as np
import rospy
import serial
import traceback 
from mobrob_util import msg
from std_msgs.msg import Float32,Float32MultiArray
from mobrob_util.msg import ME439SensorsProcessed,ME439WheelSpeeds, ME439WheelDisplacements, IMU
from geometry_msgs.msg import Twist, Pose2D
from utils import *
import math
import sys
import os
import matplotlib.pyplot as plt
import scipy.linalg as la
 
wheel_width = rospy.get_param('/wheel_width_model')

np.set_printoptions(suppress=True)

class LQR():
    def __init__(self):
        rospy.init_node('lqr_node', anonymous=False)
        self.vel_pub = rospy.Publisher('/wheel_speeds_desired', ME439WheelSpeeds, queue_size=1)
        self.vel = ME439WheelSpeeds()
        
        """#############################For Testing Purposes Only###########################"""        
        self.error_pub = rospy.Publisher('/total_error', Float32, queue_size=1)
        self.error = Float32()

        self.actual_state_pub = rospy.Publisher('/actual_state', Float32MultiArray, queue_size=1)
        self.actual_state_msg = Float32MultiArray()

        self.desired_state_pub = rospy.Publisher('/desired_state', Float32MultiArray, queue_size=1)
        self.desired_state_msg = Float32MultiArray()

        self.trajectory_pub = rospy.Publisher('/trajectory', Float32MultiArray, queue_size=1)
        self.trajectory_msg = Float32MultiArray()

        """######################################################################"""
        self.state_sub = rospy.Subscriber('/robot_pose_estimated', Pose2D, self.state_sub_callback)  
        self.actual_state_x = np.array([0,0,0])

        ax = [0.0, 0.0, 1]
        ay = [0.0, 1, 1]   
         
        # ax = np.array([0.0, 1.0, 1.25 ,1.5, 2.0, 1.5, 0.0])
        # ay = np.array([0.0, 0.5, 0.4, -0.5, 0.5, 1.0, 0.0])     
        # ax = np.array([0.0, 6.0, 12.5, 10.0, 17.5, 20.0, 25.0])*0.1
        # ay = np.array([0.0, -3.0, -5.0, 6.5, 3.0, 0.0, 0.0])*0.1

        self.desired_traj = compute_traj(ax,ay)
        self.goal = self.desired_traj[-1,:] # Coordinates of the goal
        self.goal_dis = 0.3

        self.cx, self.cy, self.cyaw, self.ck, self.s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.05)
        target_speed = 1.0 / 3.6  # simulation parameter km/h -> m/s

        self.sp = calc_speed_profile(self.cx, self.cy, cyaw=self.cyaw, target_speed=target_speed)
        self.stop_speed = 0.25
        self.v_c = 0.1
        self.ind, self.e = self.calc_nearest_index(self.actual_state_x, self.cx, self.cy, self.cyaw)
        self.total_ind = len(self.cx)
        self.total_err_lqr = 0

        self.trajectory_msg.data = np.array([self.cx, self.cy])
        self.trajectory_pub.publish(self.trajectory_msg)




    def pi_2_pi(self,angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi


    def solve_dare(self,A, B, Q, R):
        """
        solve an infinite discrete time_Algebraic Riccati equation (DARE)
        """
        x = Q
        x_next = Q
        max_iter = 150
        eps = 0.01

        for i in range(max_iter):
            x_next = A.T @ x @ A - A.T @ x @ B @ \
                    la.inv(R + B.T @ x @ B) @ B.T @ x @ A + Q
            if (abs(x_next - x)).max() < eps:
                break
            x = x_next

        return x_next

    def getB(self,yaw, deltat):
        """
        Calculates and returns the B matrix
        3x2 matix ---> number of states x number of control inputs
    
        Expresses how the state of the system [x,y,yaw] changes
        from t-1 to t due to the control commands (i.e. control inputs).
        
        :param yaw: The yaw angle (rotation angle around the z axis) in radians 
        :param deltat: The change in time from timestep t-1 to t in seconds
        
        :return: B matrix ---> 3x2 NumPy array
        """
        B = np.array([  [np.cos(yaw)*deltat, 0],
                        [np.sin(yaw)*deltat, 0],
                        [0, deltat]])
        return B
 
 
    def state_space_model(self,A, state_t_minus_1, B, control_input_t_minus_1):
        """
        Calculates the state at time t given the state at time t-1 and
        the control inputs applied at time t-1
        
        :param: A   The A state transition matrix
            3x3 NumPy Array
        :param: state_t_minus_1     The state at time t-1  
            3x1 NumPy Array given the state is [x,y,yaw angle] ---> 
            [meters, meters, radians]
        :param: B   The B state transition matrix
            3x2 NumPy Array
        :param: control_input_t_minus_1     Optimal control inputs at time t-1  
            2x1 NumPy Array given the control input vector is 
            [linear velocity of the car, angular velocity of the car]
            [meters per second, radians per second]
            
        :return: State estimate at time t
            3x1 NumPy Array given the state is [x,y,yaw angle] --->
            [meters, meters, radians]
        """
        # These next 6 lines of code which place limits on the angular and linear 
        # velocities of the robot car can be removed if you desire.
        control_input_t_minus_1[0] = np.clip(control_input_t_minus_1[0],-max_linear_velocity,max_linear_velocity)
        control_input_t_minus_1[1] = np.clip(control_input_t_minus_1[1],-max_angular_velocity,max_angular_velocity)
        state_estimate_t = (A @ state_t_minus_1) + (B @ control_input_t_minus_1) 
                
        return state_estimate_t
     

    def dlqr(self,A, B, Q, R):
        """Solve the discrete time lqr controller.
        x[k+1] = A x[k] + B u[k]
        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
        # ref Bertsekas, p.151
        """

        # first, try to solve the ricatti equation
        X = self.solve_dare(A, B, Q, R)

        # compute the LQR gain
        K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

        eig_result = la.eig(A - B @ K)

        return K, X, eig_result[0]
    
    def state_sub_callback(self, msg):
        self.x_actual = msg.x
        self.y_actual = msg.y
        self.yaw_actual = msg.theta+np.pi/2
        self.actual_state_x = np.array([self.x_actual, self.y_actual, self.yaw_actual])
        self.main()


    def controller(self):

        self.A = np.array([     [1.0,  0,   0],
                                [  0,1.0,   0],
                                [  0,  0, 1.0]])
        
        # R matrix
        # The control input cost matrix
        # Experiment with different R matrices
        # This matrix penalizes actuator effort (i.e. rotation of the 
        # motors on the wheels that drive the linear velocity and angular velocity).
        # The R matrix has the same number of rows as the number of control
        # inputs and same number of columns as the number of 
        # control inputs.
        # This matrix has positive values along the diagonal and 0s elsewhere.
        # We can target control inputs where we want low actuator effort 
        # by making the corresponding value of R large. 
        self.R = np.array([ [0.1,   0],  # Penalty for linear velocity effort
                            [  0, 0.1]]) # Penalty for angular velocity effort
    
        # Q matrix
        # The state cost matrix.
        # Experiment with different Q matrices.
        # Q helps us weigh the relative importance of each state in the 
        # state vector (X, Y, YAW ANGLE). 
        # Q is a square matrix that has the same number of rows as 
        # there are states.
        # Q penalizes bad performance.
        # Q has positive values along the diagonal and zeros elsewhere.
        # Q enables us to target states where we want low error by making the 
        # corresponding value of Q large.
        self.Q = np.array([     [1.0, 0, 0],  # Penalize X position error 
                                [0, 1.0, 0],  # Penalize Y position error 
                                [0, 0, 1.0]]) # Penalize YAW ANGLE heading error 

        self.B = np.array([     [np.cos(self.actual_state_x[2])*self.dt, 0],
                                [np.sin(self.actual_state_x[2])*self.dt, 0],
                                [0                  , self.dt]])
        
        K, _, _ = self.dlqr(self.A, self.B, self.Q, self.R)

        x_error = self.actual_state_x - self.desired_state_xf

        ustar = -K @ x_error

        return ustar
    

    def calc_nearest_index(self, state, cx, cy, cyaw):
        dx = [state[0] - icx for icx in cx]
        dy = [state[1] - icy for icy in cy]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind)

        mind = math.sqrt(mind)

        dxl = cx[ind] - state[0]
        dyl = cy[ind] - state[1]

        angle = self.pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        return ind, mind


    def main(self):

        if self.total_ind > self.ind:    

            # Let the time interval be 1.0 seconds
            self.dt = 0.1


            if abs(self.v_c) < self.stop_speed:
                self.ind += 1

            self.desired_state_xf = np.array([self.cx[self.ind], self.cy[self.ind], self.cyaw[self.ind]])  
            #replaced desired with the closest point on the trajectory

            print(f'Current State = {self.actual_state_x}')
            print(f'Desired State = {self.desired_state_xf}')
            
            state_error = self.actual_state_x - self.desired_state_xf
            state_error_magnitude = np.linalg.norm(state_error)   
            state_error_xy = np.linalg.norm(state_error[:1])    

            print(f'State Error Magnitude = {state_error_magnitude}')
            
            # LQR returns the optimal control input
            optimal_control_input = self.controller() 
            
            print(f'Control Input = {optimal_control_input}')
            self.v_c = optimal_control_input[0]
            omega = optimal_control_input[1]
            
            self.vel.v_left = (2*self.v_c-omega*wheel_width)/2
            self.vel.v_right = self.v_c+(omega*wheel_width)/2
        
            # We apply the optimal control to the robot
            # so we can get a new actual (estimated) state.
            self.actual_state_x = self.state_space_model(self.A, self.actual_state_x, self.B, 
                                            optimal_control_input)  
  
            self.vel.v_left = np.clip(self.vel.v_left,-0.2,0.6)                    
            self.vel.v_right = np.clip(self.vel.v_right,-0.2,0.6)                    

                #print()
            self.actual_state_msg.data = self.actual_state_x
            self.actual_state_pub.publish(self.actual_state_msg)

            self.desired_state_msg.data = self.desired_state_xf
            self.desired_state_pub.publish(self.desired_state_msg)
            self.total_err_lqr+=state_error_xy

            self.vel_pub.publish(self.vel)
            self.error_pub.publish(self.total_err_lqr)

        else:
            self.vel.v_left = 0.0
            self.vel.v_right = 0.0
            self.vel_pub.publish(self.vel)
            self.error_pub.publish(self.total_err_lqr)
            rospy.sleep(1000.)

# Entry point for the program
if __name__ == "__main__":
    try:
        # Optional Variables    
        max_linear_velocity = .8 # meters per second
        max_angular_velocity = 1.5708 # radians per second
        lqr = LQR()
        rospy.spin()

    except rospy.ROSInterruptException: 
        lqr.v_left = 0.0
        lqr.v_right = 0.0
        lqr.vel_pub.publish(lqr.vel)
        rospy.loginfo("LQR node terminated.")    
        pass