#!/usr/bin/env python3

# Author: Ahmed Khalil, Mohamed Safwat
# Adapted from: https://automaticaddison.com 
# Automatic Addison: https://automaticaddison.com/linear-quadratic-regulator-lqr-with-python-code-example/
 
import numpy as np
import rospy
import serial
import traceback 
from mobrob_util import msg
from std_msgs.msg import Float32,Float32MultiArray
from mobrob_util.msg import ME439SensorsProcessed,ME439WheelSpeeds, ME439WheelDisplacements, IMU
from geometry_msgs.msg import Twist, Pose2D

 
wheel_width = rospy.get_param('/wheel_width_model')

np.set_printoptions(suppress=True)

class LQR():
    def __init__(self):
        rospy.init_node('lqr_node', anonymous=False)
        self.vel_pub = rospy.Publisher('/wheel_speeds_desired', ME439WheelSpeeds, queue_size=1)
        self.vel = ME439WheelSpeeds()
        
        """######################################################################"""        
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