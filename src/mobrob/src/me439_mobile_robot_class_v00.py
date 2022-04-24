#!/usr/bin/env python3

# =============================================================================
# Peter G. Adamczyk 
# 2018-10-11
# Updated 2021-10-12
# =============================================================================

import numpy as np

class robot:
    def __init__(self,wheel_width,body_length,wheel_radius):
        # Position 
        self.r_center_world = np.array([0, 0])    # Position: initialize at 0,0
        self.theta = float(0)               # Robot angle: initialize at 0
        # Velocity
        self.v_center_world = np.array([0, 0])    # Velocity (world frame): initialize at 0,0
        self.omega = float(0)               # Robot angular velocity: initialize at 0
        # States for Wheel Speeds
        self.left_wheel_speed = 0.
        self.right_wheel_speed = 0.        
        
        self.wheel_width = wheel_width
        self.body_length = body_length
        self.wheel_radius = wheel_radius

        # Some Geometric Parameters used to draw the outline of the robot
        body_width_ratio = 0.8
        wheel_length_ratio = 0.4
        # Outline of the robot in a body-fixed frame: x Right, y Forward
        outline_bodyfixed_x = np.array( [wheel_width/2, wheel_width/2, body_width_ratio*wheel_width/2, body_width_ratio*wheel_width/2, -body_width_ratio*wheel_width/2, -body_width_ratio*wheel_width/2, -wheel_width/2, -wheel_width/2, wheel_width/2] )  # last corner closes the shape
        outline_bodyfixed_y = np.array( [body_length*(-wheel_length_ratio/2), body_length*(wheel_length_ratio/2), body_length*(wheel_length_ratio/2), body_length*(1-wheel_length_ratio/2), body_length*(1-wheel_length_ratio/2), body_length*(wheel_length_ratio/2), body_length*(wheel_length_ratio/2), body_length*(-wheel_length_ratio/2), body_length*(-wheel_length_ratio/2)] )
        self.outline_bodyfixed_xy = np.vstack( (outline_bodyfixed_x,outline_bodyfixed_y) )
        
        # Keep track of how far the wheels have gone (meters). 
        self.left_wheel_distance_traveled = 0.
        self.right_wheel_distance_traveled = 0.
        # And how far they have turned (radians)
        self.left_wheel_angle = 0.
        self.right_wheel_angle = 0. 
        
        self.position_history_world = np.array([[0.,0.]])
        
        
    def get_rotmat_body_to_world(self):
        rotmat_body_to_world = np.array([ [np.cos(self.theta), -np.sin(self.theta)],[np.sin(self.theta), np.cos(self.theta)]])
        return rotmat_body_to_world

    def get_outline_world_xy(self): 
        rotbf2w = self.get_rotmat_body_to_world()
        
        outline_rotated_xy= np.dot( rotbf2w, self.outline_bodyfixed_xy )
        outline_world_x = self.r_center_world[0] + outline_rotated_xy[0]
        outline_world_y = self.r_center_world[1] + outline_rotated_xy[1]
        outline_world_xy = np.vstack( (outline_world_x,outline_world_y) )
        return outline_world_xy

    def set_wheel_speeds(self, v_left, v_right):
        self.left_wheel_speed = v_left
        self.right_wheel_speed = v_right
        
        ## Now compute the new robot velocities based on the new wheel_speeds. 
        # put in Equations (i) and (ii) here based on self.left_wheel_speed, self.right_wheel_speed, and self.wheel_width
        v_center_y_bf = (v_left + v_right)/2.0
        v_center_bf = np.array([0, float(v_center_y_bf)])
        omega = (v_right - v_left)/self.wheel_width
        self.omega = omega
        
        # Equation vi: rotation matrix
        rotbf2w = self.get_rotmat_body_to_world()
        
        # Rotate velocity into world frame - Equation viii  (rotmat * v_center)
        self.v_center_world = np.dot( rotbf2w,  v_center_bf)        

    def integration_step(self, dt):
        # Structure: Take the old position and add v*dt to get a new position. Take the old angle and add omega*dt to get new angle. Then update v and omega with the new wheel speed settings. 
#        NOTE here we do the integration using the Past values of the velocities, and the new dt (how long that has been going)
        # FUTURE: use Trapezoidal integration: mean of Past value and New value of wheel speeds
        
        # Kinematics of the integration - Equations xii, xiii
        self.r_center_world = self.r_center_world + self.v_center_world*dt     # vector addition
        self.theta = self.theta + self.omega*dt
        
        # Also update the wheels' distance moved. 
        self.left_wheel_distance_traveled += self.left_wheel_speed*dt
        self.right_wheel_distance_traveled += self.right_wheel_speed*dt
        
        # And update the wheels' angle moved
        self.left_wheel_angle = self.left_wheel_distance_traveled / self.wheel_radius
        self.right_wheel_angle = self.right_wheel_distance_traveled / self.wheel_radius

    def append_current_position_to_history(self): 
        self.position_history_world = np.vstack((self.position_history_world, self.r_center_world))

   
