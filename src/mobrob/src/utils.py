"""
Program: utils.py
This program helps calculate the waypoints along the racetrack
that the robot needs to follow.
 
Modified from code developed by Atsushi Sakai
Source: https://github.com/AtsushiSakai/PythonRobotics
"""
import numpy as np
import math
import cubic_spline_planner
 
def pi_2_pi(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi
 
def compute_traj(ax,ay):
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)
    target_speed = 1  # simulation parameter km/h -> m/s
    sp = calc_speed_profile(cx, cy, cyaw, target_speed)
 
    desired_traj = np.array([cx,cy,cyaw,sp]).T
    return desired_traj
 
def calc_nearest_index(state, traj):
    cx = traj[:,0]
    cy = traj[:,1]
    cyaw = traj[:,2]
    dx = [state[0] - icx for icx in cx]
    dy = [state[1] - icy for icy in cy]
 
    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]
 
    mind = min(d)
 
    ind = d.index(mind)
 
    mind = math.sqrt(mind)
 
    dxl = cx[ind] - state[0]
    dyl = cy[ind] - state[1]
 
    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1
 
    return ind, mind
 
 
def calc_speed_profile(cx, cy, cyaw, target_speed):
    speed_profile = [target_speed] * len(cx)
 
    direction = 1.0
 
    # Set stop point
    for i in range(len(cx) - 1):
        dyaw = abs(cyaw[i + 1] - cyaw[i])
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0
 
        if switch:
            direction *= -1
 
        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed
 
        if switch:
            speed_profile[i] = 0.0
 
    speed_profile[-1] = 0.0
 
    return speed_profile