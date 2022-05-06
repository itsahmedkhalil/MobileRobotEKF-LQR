"""
A robot will follow a racetrack using an LQR controller and EKF (with landmarks)
to estimate the state (i.e. x position, y position, yaw angle) at each timestep
 
####################
 
Adapted from code authored by Atsushi Sakai (@Atsushi_twi)
Source: https://github.com/AtsushiSakai/PythonRobotics
 
"""
# Import important libraries
import numpy as np
import math
import matplotlib.pyplot as plt
from kinematics import *
from sensors import *
from utils import *
from time import sleep
 
show_animation = True
 
def closed_loop_prediction(desired_traj, landmarks):
 
    ## Simulation Parameters
    T = desired_traj.shape[0]  # Maximum simulation time
    goal_dis = 0.1 # How close we need to get to the goal
    goal = desired_traj[-1,:] # Coordinates of the goal
    dt = 0.1 # Timestep interval
    time = 0.0 # Starting time
 
 
    ## Initial States 
    state = np.array([8.3,0.69,0]) # Initial state of the car
    state_est = state.copy()
    sigma = 0.1*np.ones(3) # State covariance matrix
 
    ## Get the Cost-to-go and input cost matrices for LQR
    Q = get_Q() # Defined in kinematics.py
    R = get_R() # Defined in kinematics.py
 
 
    ## Initialize the Car and the Car's landmark sensor 
    DiffDrive = DifferentialDrive()
    LandSens = LandmarkDetector(landmarks)
         
    # Process noise and sensor measurement noise
    V = DiffDrive.get_V()
    W = LandSens.get_W()
 
    ## Create objects for storing states and estimated state
    t = [time]
    traj = np.array([state])
    traj_est = np.array([state_est])
 
    ind = 0
    while T >= time:
         
        ## Point to track
        ind = int(np.floor(time))
        goal_i = desired_traj[ind,:]
 
        ## Generate noise
        # v = process noise, w = measurement noise
        v = np.random.multivariate_normal(np.zeros(V.shape[0]),V)
        w = np.random.multivariate_normal(np.zeros(W.shape[0]),W)
 
        ## Generate optimal control commands
        u_lqr = dLQR(DiffDrive,Q,R,state_est,goal_i[0:3],dt)
 
        ## Move forwad in time
        state = DiffDrive.forward(state,u_lqr,v,dt)
                 
        # Take a sensor measurement for the new state
        y = LandSens.measure(state,w)
 
        # Update the estimate of the state using the EKF
        state_est, sigma = EKF(DiffDrive,LandSens,y,state_est,sigma,u_lqr,dt)
 
        # Increment time
        time = time + dt
 
        # Store the trajectory and estimated trajectory
        t.append(time)
        traj = np.concatenate((traj,[state]),axis=0)
        traj_est = np.concatenate((traj_est,[state_est]),axis=0)
 
        # Check to see if the robot reached goal
        if np.linalg.norm(state[0:2]-goal[0:2]) <= goal_dis:
            print("Goal reached")
            break
 
        ## Plot the vehicles trajectory
        if time % 1 < 0.1 and show_animation:
            plt.cla()
            plt.plot(desired_traj[:,0], desired_traj[:,1], "-r", label="course")
            plt.plot(traj[:,0], traj[:,1], "ob", label="trajectory")
            plt.plot(traj_est[:,0], traj_est[:,1], "sk", label="estimated trajectory")
 
            plt.plot(goal_i[0], goal_i[1], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("SINGAPORE GRAND PRIX\n" + "speed[m/s]:" + str(
                                            round(np.mean(u_lqr), 2)) +
                      ",target index:" + str(ind))
            plt.pause(0.0001)
 
        #input()
 
    return t, traj
 
 
def main():
 
    # Countdown to start
    print("\n*** SINGAPORE GRAND PRIX ***\n")
    print("Start your engines!")
    print("3.0")
    sleep(1.0)
    print("2.0")
    sleep(1.0)
    print("1.0\n")
    sleep(1.0)
    print("LQR + EKF steering control tracking start!!")
 
    # Create the track waypoints
    ax = [8.3,8.0, 7.2, 6.5, 6.2, 6.5, 1.5,-2.0,-3.5,-5.0,-7.9,
       -6.7,-6.7,-5.2,-3.2,-1.2, 0.0, 0.2, 2.5, 2.8, 4.4, 4.5, 7.8, 8.5, 8.3]
    ay = [0.7,4.3, 4.5, 5.2, 4.0, 0.7, 1.3, 3.3, 1.5, 3.0,-1.0,
       -2.0,-3.0,-4.5, 1.1,-0.7,-1.0,-2.0,-2.2,-1.2,-1.5,-2.4,-2.7,-1.7,-0.1]
     
    # These landmarks help the mobile robot localize itself
    landmarks = [[4,3],
                 [8,2],
                 [-1,-4]]
         
    # Compute the desired trajectory
    desired_traj = compute_traj(ax,ay)
 
    t, traj = closed_loop_prediction(desired_traj,landmarks)
 
    # Display the trajectory that the mobile robot executed
    if show_animation:
        plt.close()
        flg, _ = plt.subplots(1)
        plt.plot(ax, ay, "xb", label="input")
        plt.plot(desired_traj[:,0], desired_traj[:,1], "-r", label="spline")
        plt.plot(traj[:,0], traj[:,1], "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()
 
        plt.show()
 
 
if __name__ == '__main__':
    main()