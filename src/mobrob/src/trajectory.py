#!/usr/bin/env python3

"""
A robot will follow a racetrack using an LQR controller and EKF (with landmarks)
to estimate the state (i.e. x position, y position, yaw angle) at each timestep
 
####################
 
Adapted from code authored by Atsushi Sakai (@Atsushi_twi)
Source: https://github.com/AtsushiSakai/PythonRobotics
 
"""
# Import important libraries
import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
from time import sleep
import serial
import traceback 
from kinematics import *
from sensors import *
from utils import *
from mobrob_util import msg
from std_msgs.msg import Float32,Float32MultiArray
from mobrob_util.msg import ME439SensorsProcessed,ME439WheelSpeeds, ME439WheelDisplacements, IMU
from geometry_msgs.msg import Twist, Pose2D
 
show_animation = True
wheel_width = rospy.get_param('/wheel_width_model')

# Author: Addison Sears-Collins
# https://automaticaddison.com
# Description: Linear Quadratic Regulator example 
#   (two-wheeled differential drive robot car)
 
######################## DEFINE CONSTANTS #####################################
# Supress scientific notation when printing NumPy arrays
np.set_printoptions(suppress=True)

class LQR():
    def __init__(self):
        rospy.init_node('lqr_node', anonymous=False)
        self.vel_pub = rospy.Publisher('/wheel_speeds_desired', ME439WheelSpeeds, queue_size=1)
        self.vel = ME439WheelSpeeds()
        
        """#######################for debugging, feel free to comment##############################"""        
        self.error_pub = rospy.Publisher('/total_error', Float32, queue_size=1)
        self.error = Float32()

        self.actual_state_pub = rospy.Publisher('/actual_state', Float32MultiArray, queue_size=1)
        self.actual_state_msg = Float32MultiArray()

        self.desired_state_pub = rospy.Publisher('/desired_state', Float32MultiArray, queue_size=1)
        self.desired_state_msg = Float32MultiArray()

        """######################################################################"""
        self.state_sub = rospy.Subscriber('/robot_pose_estimated', Pose2D, self.state_sub_callback)  
        self.actual_state_x = np.array([0,0,0])
        # self.timer_period = 0.05  # seconds
        # self.timer = self.create_timer(self.timer_period, self.loop)

    def forward(self,x0,u,dt=0.1):
        """
        Computes the forward kinematics for the system.
    
        Input
        :param x0: The starting state (position) of the system (units:[m,m,rad])  
                    np.array with shape (3,) -> 
                    (X, Y, THETA)
        :param u:  The control input to the system  
                    2x1 NumPy Array given the control input vector is  
                    [linear velocity of the car, angular velocity of the car] 
                    [meters per second, radians per second]
        :param dt: Change in time (units: [s])
    
        Output
        :return: x1: The new state of the system (X, Y, THETA)
        """
        u0 = u 
            
            # Starting state of the vehicle
        X = x0[0]
        Y = x0[1]
        THETA = x0[2]
    
        # Control input
        u_linvel = u0[0]
        u_angvel = u0[1]
    
        # Velocity in the x and y direction in m/s
        x_dot = u_linvel * np.cos(THETA)
        y_dot = u_linvel * np.sin(THETA)
    
        # The new state of the system
        x1 = np.empty(3)
        
        # Calculate the new state of the system
        # Noise is added like in slide 34 in Lecture 7
        x1[0] = x0[0] + x_dot * dt # X
        x1[1] = x0[1] + y_dot * dt # Y
        x1[2] = x0[2] + u_angvel * dt # THETA
    
        return x1

    def linearize(self, x, dt=0.1):
        """
        Creates a linearized version of the dynamics of the differential 
        drive robotic system (i.e. a
        robotic car where each wheel is controlled separately.
    
        The system's forward kinematics are nonlinear due to the sines and 
        cosines, so we need to linearize 
        it by taking the Jacobian of the forward kinematics equations with respect 
        to the control inputs.
    
        Our goal is to have a discrete time system of the following form: 
        x_t+1 = Ax_t + Bu_t where:
    
        Input
        :param x: The state of the system (units:[m,m,rad]) -> 
                    np.array with shape (3,) ->
                    (X, Y, THETA) ->
                    X_system = [x1, x2, x3]
        :param dt: The change in time from time step t to time step t+1      
    
        Output
        :return: A: Matrix A is a 3x3 matrix (because there are 3 states) that 
                    describes how the state of the system changes from t to t+1 
                    when no control command is executed. Typically, 
                    a robotic car only drives when the wheels are turning. 
                    Therefore, in this case, A is the identity matrix.
        :return: B: Matrix B is a 3 x 2 matrix (because there are 3 states and 
                    2 control inputs) that describes how
                    the state (X, Y, and THETA) changes from t to t + 1 due to 
                    the control command u.
                    Matrix B is found by taking the The Jacobian of the three 
                    forward kinematics equations (for X, Y, THETA) 
                    with respect to u (3 x 2 matrix)
    
        """
        THETA = x[2]
    
        ####### A Matrix #######
        # A matrix is the identity matrix
        A = np.array([[1.0,   0,  0],
                    [  0, 1.0,  0],
                    [  0,   0, 1.0]])
    
        ####### B Matrix #######
        B = np.array([[np.cos(THETA)*dt, 0],
                    [np.sin(THETA)*dt, 0],
                    [0, dt]])
            
        return A, B

    def dLQR(F,Q,R,x,xf,dt=0.1):
        """
        Discrete-time linear quadratic regulator for a non-linear system.
    
        Compute the optimal control given a nonlinear system, cost matrices, 
        a current state, and a final state.
        Compute the control variables that minimize the cumulative cost.
    
        Solve for P using the dynamic programming method.
    
        Assume that Qf = Q
    
        Input:
        :param F: The dynamics class object (has forward and linearize functions 
                    implemented)
        :param Q: The state cost matrix Q -> np.array with shape (3,3)
        :param R: The input cost matrix R -> np.array with shape (2,2)
        :param x: The current state of the system x -> np.array with shape (3,)
        :param xf: The desired state of the system xf -> np.array with shape (3,)
        :param dt: The size of the timestep -> float
    
        Output
        :return: u_t_star: Optimal action u for the current state 
                    [linear velocity of the car, angular velocity of the car]
                    [meters per second, radians per second]
        """
        # We want the system to stabilize at xf, 
        # so we let x - xf be the state.
        # Actual state - desired state
        x_error = x - xf
    
        # Calculate the A and B matrices
        A, B = F.linearize(x, dt)
    
        # Solutions to discrete LQR problems are obtained using dynamic 
        # programming.
        # The optimal solution is obtained recursively, starting at the last 
        # time step and working backwards.
        N = 50
    
        # Create a list of N + 1 elements
        P = [None] * (N + 1)
    
        # Assume Qf = Q
        Qf = Q
    
        # 1. LQR via Dynamic Programming 
        P[N] = Qf 
    
        # 2. For t = N, ..., 1
        for t in range(N, 0, -1):
    
            # Discrete-time Algebraic Riccati equation to calculate the optimal 
            # state cost matrix
            P[t-1] = Q + A.T @ P[t] @ A - (A.T @ P[t] @ B) @ la.pinv(
                    R + B.T @ P[t] @ B) @ (B.T @ P[t] @ A)      
    
        # Create a list of N elements
        K = [None] * N
        u = [None] * N
    
        # 3 and 4. For t = 0, ..., N - 1
        for t in range(N):
    
            # Calculate the optimal feedback gain K_t
            K[t] = -la.pinv(R + B.T @ P[t+1] @ B) @ B.T @ P[t+1] @ A
        
        for t in range(N):
        
            # Calculate the optimal control input
            u[t] = K[t] @ x_error
        
            # Optimal control input is u_t_star
            u_t_star = u[N-1]
    
        # Return the optimal control inputs
        return u_t_star


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
     
    def lqr(self,actual_state_x, desired_state_xf, Q, R, A, B, dt):
        """
        Discrete-time linear quadratic regulator for a nonlinear system.
    
        Compute the optimal control inputs given a nonlinear system, cost matrices, 
        current state, and a final state.
        
        Compute the control variables that minimize the cumulative cost.
    
        Solve for P using the dynamic programming method.
    
        :param actual_state_x: The current state of the system 
            3x1 NumPy Array given the state is [x,y,yaw angle] --->
            [meters, meters, radians]
        :param desired_state_xf: The desired state of the system
            3x1 NumPy Array given the state is [x,y,yaw angle] --->
            [meters, meters, radians]   
        :param Q: The state cost matrix
            3x3 NumPy Array
        :param R: The input cost matrix
            2x2 NumPy Array
        :param dt: The size of the timestep in seconds -> float
    
        :return: u_star: Optimal action u for the current state 
            2x1 NumPy Array given the control input vector is
            [linear velocity of the car, angular velocity of the car]
            [meters per second, radians per second]
        """
        # We want the system to stabilize at desired_state_xf.
        x_error = actual_state_x - desired_state_xf
    
        # Solutions to discrete LQR problems are obtained using the dynamic 
        # programming method.
        # The optimal solution is obtained recursively, starting at the last 
        # timestep and working backwards.
        # You can play with this number
        N = 50
    
        # Create a list of N + 1 elements
        P = [None] * (N + 1)
        
        Qf = Q
    
        # LQR via Dynamic Programming
        P[N] = Qf
    
        # For i = N, ..., 1
        for i in range(N, 0, -1):
    
            # Discrete-time Algebraic Riccati equation to calculate the optimal 
            # state cost matrix
            P[i-1] = Q + A.T @ P[i] @ A - (A.T @ P[i] @ B) @ np.linalg.pinv(
                R + B.T @ P[i] @ B) @ (B.T @ P[i] @ A)      
    
        # Create a list of N elements
        K = [None] * N
        u = [None] * N
    
        # For i = 0, ..., N - 1
        for i in range(N):
    
            # Calculate the optimal feedback gain K
            K[i] = -np.linalg.pinv(R + B.T @ P[i+1] @ B) @ B.T @ P[i+1] @ A
    
            u[i] = K[i] @ x_error
    
        # Optimal control input is u_star
        u_star = u[N-1]
    
        return u_star
    
    def state_sub_callback(self, msg):
        """
        Callback function for the subscriber to the /mobrob/odom topic.
        """
        self.x_actual = msg.x
        self.y_actual = msg.y
        self.yaw_actual = msg.theta+np.pi/2
        #print(self.yaw_actual)
        self.loop()

 
    def loop(self):
        
        # Let the time interval be 1.0 seconds
        dt = 0.1
        
        # Actual state
        # Our robot starts out at the origin (x=0 meters, y=0 meters), and 
        # the yaw angle is 0 radians. 
        #actual_state_x = np.array([0,0,0])
        self.actual_state_x = np.array([self.x_actual,self.y_actual,self.yaw_actual]) 
    
        # Desired state [x,y,yaw angle]
        # [meters, meters, radians]
        desired_state_xf = np.array([1.0,1.0,np.pi/4])  
        
        # A matrix
        # 3x3 matrix -> number of states x number of states matrix
        # Expresses how the state of the system [x,y,yaw] changes 
        # from t-1 to t when no control command is executed.
        # Typically a robot on wheels only drives when the wheels are told to turn.
        # For this case, A is the identity matrix.
        # Note: A is sometimes F in the literature.
        A = np.array([  [1.0,  0,   0],
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
        R = np.array([[0.1,   0],  # Penalty for linear velocity effort
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
        Q = np.array([[1.0, 0, 0],  # Penalize X position error 
                                    [0, 1.0, 0],  # Penalize Y position error 
                                    [0, 0, 1.0]]) # Penalize YAW ANGLE heading error 
                    
        # Launch the robot, and have it move to the desired goal destination
        #for i in range(100):
        #print(f'iteration = {i} seconds')
        print(f'Current State = {self.actual_state_x}')
        print(f'Desired State = {desired_state_xf}')
        
        state_error = self.actual_state_x - desired_state_xf
        state_error_magnitude = np.linalg.norm(state_error)     
        print(f'State Error Magnitude = {state_error_magnitude}')
        
        B = self.getB(self.actual_state_x[2], dt)
        
        # LQR returns the optimal control input
        optimal_control_input = self.lqr(self.actual_state_x, 
                                    desired_state_xf, 
                                    Q, R, A, B, dt) 
        
        print(f'Control Input = {optimal_control_input}')
        v_c = optimal_control_input[0]
        omega = optimal_control_input[1]
        
        self.vel.v_left = (2*v_c-omega*wheel_width)/2
        self.vel.v_right = v_c+(omega*wheel_width)/2
        
        # We apply the optimal control to the robot
        # so we can get a new actual (estimated) state.
        self.actual_state_x = self.state_space_model(A, self.actual_state_x, B, 
                                        optimal_control_input)  
        if state_error_magnitude < 0.1:
            self.vel.v_left = 0.0
            self.vel.v_right = 0.0
            # Stop as soon as we reach the goal
            # Feel free to change this threshold value.
            # if state_error_magnitude < 0.01:
            #     print("\nGoal Has Been Reached Successfully!")
            #     break
        self.vel.v_left = np.clip(self.vel.v_left,-0.2,0.3)                    
        self.vel.v_right = np.clip(self.vel.v_right,-0.2,0.3)                    

            #print()
        self.actual_state_msg.data = self.actual_state_x
        self.actual_state_pub.publish(self.actual_state_msg)

        self.desired_state_msg.data = desired_state_xf
        self.desired_state_pub.publish(self.desired_state_msg)

        self.vel_pub.publish(self.vel)
        self.error_pub.publish(state_error_magnitude)



# Entry point for the program
if __name__ == "__main__":
    try:
        # Optional Variables    
        max_linear_velocity = .4 # meters per second
        max_angular_velocity = 1.5708 # radians per second
        lqr = LQR()
        rospy.spin()

    except rospy.ROSInterruptException: 
        lqr.v_left = 0.0
        lqr.v_right = 0.0
        lqr.vel_pub.publish(lqr.vel)
        rospy.loginfo("LQR node terminated.")    
        pass