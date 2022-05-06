""" 
Implementation of the two-wheeled differential drive robot car 
and its controller.
 
Our goal in using LQR is to find the optimal control inputs:
  [linear velocity of the car, angular velocity of the car]
     
We want to both minimize the error between the current state 
and a desired state, while minimizing actuator effort 
(e.g. wheel rotation rate). These are competing objectives because a 
large u (i.e. wheel rotation rates) expends a lot of
actuator energy but can drive the state error to 0 really fast.
LQR helps us balance these competing objectives.
 
If a system is linear, LQR gives the optimal control inputs that 
takes a system's state to 0, where the state is
"current state - desired state".
 
Implemented by Addison Sears-Collins
Date: December 10, 2020
 
"""
# Import required libraries
import numpy as np
import scipy.linalg as la
 
class DifferentialDrive(object):
  """
  Implementation of Differential Drive kinematics.
  This represents a two-wheeled vehicle defined by the following states
  state = [x,y,theta] where theta is the yaw angle
  and accepts the following control inputs
  input = [linear velocity of the car, angular velocity of the car]
  """
  def __init__(self):
    """
    Initializes the class
    """
    # Covariance matrix representing action noise 
    # (i.e. noise on the control inputs)
    self.V = None
 
  def get_state_size(self):
    """
    The state is (X, Y, THETA) in global coordinates, so the state size is 3.
    """
    return 3
 
  def get_input_size(self):
    """
    The control input is ([linear velocity of the car, angular velocity of the car]), 
    so the input size is 2.
    """
    return 2
 
  def get_V(self):
    """
    This function provides the covariance matrix V which 
    describes the noise that can be applied to the forward kinematics.
     
    Feel free to experiment with different levels of noise.
 
    Output
      :return: V: input cost matrix (3 x 3 matrix)
    """
    # The np.eye function returns a 2D array with ones on the diagonal
    # and zeros elsewhere.
    if self.V is None:
      self.V = np.eye(3)
      self.V[0,0] = 0.01
      self.V[1,1] = 0.01
      self.V[2,2] = 0.1
    return 1e-5*self.V
 
  def forward(self,x0,u,v,dt=0.1):
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
      :param v:  The noise applied to the system (units:[m, m, rad]) ->np.array 
                 with shape (3,)
      :param dt: Change in time (units: [s])
 
    Output
      :return: x1: The new state of the system (X, Y, THETA)
    """
    u0 = u 
         
    # If there is no noise applied to the system
    if v is None:
      v = 0
     
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
    x1[0] = x0[0] + x_dot * dt + v[0] # X
    x1[1] = x0[1] + y_dot * dt + v[1] # Y
    x1[2] = x0[2] + u_angvel * dt + v[2] # THETA
 
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
 
def get_R():
    """
    This function provides the R matrix to the lqr_ekf_control simulator.
     
    Returns the input cost matrix R.
 
    Experiment with different gains.
    This matrix penalizes actuator effort 
    (i.e. rotation of the motors on the wheels).
    The R matrix has the same number of rows as are actuator states 
    [linear velocity of the car, angular velocity of the car]
    [meters per second, radians per second]
    This matrix often has positive values along the diagonal.
    We can target actuator states where we want low actuator 
    effort by making the corresponding value of R large.   
 
    Output
      :return: R: Input cost matrix
    """
    R = np.array([[0.01, 0],  # Penalization for linear velocity effort
                  [0, 0.01]]) # Penalization for angular velocity effort
 
    return R
 
def get_Q():
    """
    This function provides the Q matrix to the lqr_ekf_control simulator.
     
    Returns the state cost matrix Q.
 
    Experiment with different gains to see their effect on the vehicle's 
    behavior.
    Q helps us weight the relative importance of each state in the state 
    vector (X, Y, THETA). 
    Q is a square matrix that has the same number of rows as there are states.
    Q penalizes bad performance.
    Q has positive values along the diagonal and zeros elsewhere.
    Q enables us to target states where we want low error by making the 
    corresponding value of Q large.
    We can start with the identity matrix and tweak the values through trial 
    and error.
 
    Output
      :return: Q: State cost matrix (3x3 matrix because the state vector is 
                  (X, Y, THETA))
    """
    Q = np.array([[0.4, 0, 0], # Penalize X position error (global coordinates)
                  [0, 0.4, 0], # Penalize Y position error (global coordinates)
                  [0, 0, 0.4]])# Penalize heading error (global coordinates)
     
    return Q