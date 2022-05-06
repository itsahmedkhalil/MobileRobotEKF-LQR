"""
Extended Kalman Filter implementation using landmarks to localize
 
Implemented by Addison Sears-Collins
Date: December 10, 2020
"""
import numpy as np
import scipy.linalg as la
import math
 
class LandmarkDetector(object):
  """
  This class represents the sensor mounted on the robot that is used to 
  to detect the location of landmarks in the environment.
  """
  def __init__(self,landmark_list):
    """
    Calculates the sensor measurements for the landmark sensor
         
    Input
      :param landmark_list: 2D list of landmarks [L1,L2,L3,...,LN], 
                            where each row is a 2D landmark defined by 
                                        Li =[l_i_x,l_i_y]
                            which corresponds to its x position and y 
                            position in the global coordinate frame.
    """
    # Store the x and y position of each landmark in the world
    self.landmarks = np.array(landmark_list) 
 
    # Record the total number of landmarks
    self.N_landmarks = self.landmarks.shape[0]
         
    # Variable that represents landmark sensor noise (i.e. error)
    self.W = None
 
  def get_W(self):
    """
    This function provides the covariance matrix W which describes the noise 
    that can be applied to the sensor measurements.
 
    Feel free to experiment with different levels of noise.
     
    Output
      :return: W: measurement noise matrix (4x4 matrix given two landmarks)
                  In this implementation there are two landmarks. 
                  Each landmark has an x location and y location, so there are 
                  4 individual sensor measurements. 
    """
    # In the EKF code, we will condense this 4x4 matrix so that it is a 4x1 
    # landmark sensor measurement noise vector (i.e. [x1 noise, y1 noise, x2 
    # noise, y2 noise]
    if self.W is None:
      self.W = 0.0095*np.eye(2*self.N_landmarks)
    return self.W
 
  def measure(self,x,w=None):
    """
    Computes the landmark sensor measurements for the system. 
    This will be a list of range (i.e. distance) 
    and relative angles between the robot and a 
    set of landmarks defined with [x,y] positions.
 
    Input
      :param x: The state [x_t,y_t,theta_t] of the system -> np.array with 
                shape (3,). x is a 3x1 vector
      :param w: The noise (assumed Gaussian) applied to the measurement -> 
                np.array with shape (2 * N_Landmarks,)
                w is a 4x1 vector
    Output
      :return: y: The resulting observation vector (4x1 in this 
                  implementation because there are 2 landmarks). The 
                  resulting measurement is 
                  y = [h1_range_1,h1_angle_1, h2_range_2, h2_angle_2]
    """
    # Create the observation vector
    y = np.zeros(shape=(2*self.N_landmarks,))
 
    # Initialize state variables    
    x_t = x[0]
    y_t = x[1]
    THETA_t = x[2]
     
    # Fill in the observation model y
    for i in range(0, self.N_landmarks):
 
      # Set the x and y position for landmark i
      x_l_i = self.landmarks[i][0]
      y_l_i = self.landmarks[i][1]
 
      # Set the noise value
      w_r = w[i*2]
      w_b = w[i*2+1]      
 
      # Calculate the range (distance from robot to the landmark) and 
            # bearing angle to the landmark         
      range_i = math.sqrt(((x_t - x_l_i)**2) + ((y_t - y_l_i)**2)) + w_r
      angle_i = math.atan2(y_t - y_l_i,x_t - x_l_i) - THETA_t + w_b
 
      # Populate the predicted sensor observation vector      
      y[i*2] = range_i
      y[i*2+1] = angle_i
 
    return y
 
  def jacobian(self,x):
    """
    Computes the first order jacobian around the state x
 
    Input
      :param x: The starting state (position) of the system -> np.array 
                with shape (3,)
 
    Output
      :return: H: The resulting Jacobian matrix H for the sensor. 
                  The number of rows is equal to two times the number of 
                  landmarks H. 2 rows (for the range (distance) and bearing 
                  to landmark measurement) x 3 columns for the number of 
                  states (X, Y, THETA).
   
                    In other words, the number of rows is equal to the total 
                  number of individual sensor measurements.
                          Since we have 2 landmarks, H will be a 4x3 matrix 
                    (2 * N_Landmarks x Number of states)      
    """
    # Create the Jacobian matrix H
    # For example, the first two rows of the Jacobian matrix H pertain to 
    # the first landmark:      
    #   dh_range/dx_t    dh_range/dy_t    dh_range/dtheta_t
    #   dh_bearing/dx_t  dh_bearing/dy_t  dh_bearing/dtheta_t
    # Note that 'bearing' and 'angle' are synonymous in this program.
    H = np.zeros(shape=(2 * self.N_landmarks, 3))
 
    # Extract the state
    X_t = x[0]
    Y_t = x[1]
    THETA_t = x[2]
    
    # Fill in H, the Jacobian matrix with the partial derivatives of the 
    # observation (measurement) model h
    # Partial derivatives were calculated using an online partial derivatives 
    # calculator.
    # atan2 derivatives are listed here https://en.wikipedia.org/wiki/Atan2
    for i in range(0, self.N_landmarks):
     
      # Set the x and y position for landmark i
      x_l_i = self.landmarks[i][0]
      y_l_i = self.landmarks[i][1]
       
      dh_range_dx_t = (X_t-x_l_i) / math.sqrt((X_t-x_l_i)**2 + (Y_t-y_l_i)**2)
      H[i*2][0] = dh_range_dx_t      
 
      dh_range_dy_t = (Y_t-y_l_i) / math.sqrt((X_t-x_l_i)**2 + (Y_t-y_l_i)**2)
      H[i*2][1] = dh_range_dy_t     
 
      dh_range_dtheta_t = 0
      H[i*2][2] = dh_range_dtheta_t
 
      dh_bearing_dx_t = -(Y_t-y_l_i) / ((X_t-x_l_i)**2 + (Y_t-y_l_i)**2)
      H[i*2+1][0] = dh_bearing_dx_t   
       
      dh_bearing_dy_t = (X_t-x_l_i) / ((X_t-x_l_i)**2 + (Y_t-y_l_i)**2)
      H[i*2+1][1] = dh_bearing_dy_t    
 
      dh_bearing_dtheta_t = -1.0  
      H[i*2+1][2] = dh_bearing_dtheta_t
 
    return H
 
def EKF(DiffDrive,Sensor,y,x_hat,sigma,u,dt,V=None,W=None):
    """
    Some of these matrices will be non-square or singular. Utilize the 
    pseudo-inverse function la.pinv instead of inv to avoid these errors.
 
    Input
      :param DiffDrive: The DifferentialDrive object defined in kinematics.py
      :param Sensor: The Landmark Sensor object defined in this class
      :param y: The observation vector (4x1 in this implementation because 
                there are 2 landmarks). The measurement is 
                y = [h1_range_1,h1_angle_1, h2_range_2, h2_angle_2]
      :param x_hat: The starting estimate of the state at time t -> 
                    np.array with shape (3,)
                    (X_t, Y_t, THETA_t)
      :param sigma: The state covariance matrix at time t -> np.array with 
                    shape (3,1) initially, then 3x3
      :param u: The input to the system at time t -> np.array with shape (2,1)
                These are the control inputs to the system.
                [left wheel rotational velocity, right wheel rotational 
                velocity]
      :param dt: timestep size delta t
      :param V: The state noise covariance matrix  -> np.array with shape (3,3)
      :param W: The measurment noise covariance matrix -> np.array with shape 
                (2*N_Landmarks,2*N_Landmarks)
                4x4 matrix
 
    Output
      :return: x_hat_2: The estimate of the state at time t+1 
                        [X_t+1, Y_t+1, THETA_t+1]
      :return: sigma_est: The new covariance matrix at time t+1 (3x3 matrix)
       
    """
    V = DiffDrive.get_V() # 3x3 matrix for the state noise
    W = Sensor.get_W() # 4x4 matrix for the measurement noise
 
    ## Generate noise
    # v = process noise, w = measurement noise
    v = np.random.multivariate_normal(np.zeros(V.shape[0]),V) # 3x1 vector
    w = np.random.multivariate_normal(np.zeros(W.shape[0]),W) # 4x1 vector  
     
    ##### Prediction Step #####
 
    # Predict the state estimate based on the previous state and the 
    # control input
    # x_predicted is a 3x1 vector (X_t+1, Y_t+1, THETA_t+1)
    x_predicted = DiffDrive.forward(x_hat,u,v,dt)
 
    # Calculate the A and B matrices    
    A, B = DiffDrive.linearize(x=x_hat)
     
    # Predict the covariance estimate based on the 
    # previous covariance and some noise
    # A and V are 3x3 matrices
    sigma_3by3 = None
    if (sigma.size == 3):
      sigma_3by3 = sigma * np.eye(3)
    else:
      sigma_3by3 = sigma
       
    sigma_new = A @ sigma_3by3 @ A.T + V
 
    ##### Correction Step #####  
 
    # Get H, the 4x3 Jacobian matrix for the sensor
    H = Sensor.jacobian(x_hat) 
 
    # Calculate the observation model   
    # y_predicted is a 4x1 vector
    y_predicted = Sensor.measure(x_predicted, w)
 
    # Measurement residual (delta_y is a 4x1 vector)
    delta_y = y - y_predicted
 
    # Residual covariance 
    # 4x3 @ 3x3 @ 3x4 -> 4x4 matrix
    S = H @ sigma_new @ H.T + W
 
    # Compute the Kalman gain
    # The Kalman gain indicates how certain you are about
    # the observations with respect to the motion
    # 3x3 @ 3x4 @ 4x4 -> 3x4
    K = sigma_new @ H.T @ la.pinv(S)
 
    # Update the state estimate
    # 3x1 + (3x4 @ 4x1 -> 3x1)
    x_hat_2 = x_predicted + (K @ delta_y)
     
    # Update the covariance estimate
    # 3x3 - (3x4 @ 4x3) @ 3x3
    sigma_est = sigma_new - (K @ H @ sigma_new)
 
    return x_hat_2 , sigma_est
