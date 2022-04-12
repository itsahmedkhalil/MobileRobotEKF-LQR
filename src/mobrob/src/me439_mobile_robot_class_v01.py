#!/usr/bin/env python3

# =============================================================================
# Peter G. Adamczyk 
# Updated 2019-10-12
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

#### CODE HERE
    def set_wheel_speeds_from_robot_velocities(self, forward_velocity, angular_velocity):
        ### Kinematic Equations mapping desired robot speed and angular velocity
        ### to left and right wheel speeds. 
        ### Use the variables "forward_velocity" and "angular_velocity" that the function takes as input. 
        ### Use "self.wheel_width" as the robot's parameter for wheel width. 
        left_wheel_speed = forward_velocity - angular_velocity*self.wheel_width/2 
        right_wheel_speed = forward_velocity + angular_velocity*self.wheel_width/2 
        
        self.set_wheel_speeds(left_wheel_speed, right_wheel_speed)
#### END CODE
        
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

   
####    CODE HERE: -- ALL FUNCTIONS BELOW NEED EDITING   
# =============================================================================
# # PATH PLANNING for Open-Loop Control
# # Accept required path segment as a line, arc or pivot 
# # and return a list specifying [duration, left_wheel_speed, right_wheel_speed]
# # that will accomplish that path segment. 
# #         
# # These functions will be called as: 
# #    robot.plan_line(speed, length)
# #    robot.plan_arc(radius, speed, angle)
# #    robot.plan_pivot(omega, angle)
# =============================================================================
      
# =============================================================================
#     # Plan a line: accept input of speed (m/s) and line segment length (m). 
#     # Return [duration (s), left_wheel_speed (m/s), right_wheel_speed (m/s)]
# =============================================================================
    def plan_line(self, speed, length):
        # Trick: correct sign mismatches between speed and length
        speed = np.abs(speed)*np.sign(length)
        
        # Compute the needed wheel speeds to go straight: 
        lft_whl_spd = speed
        rgt_whl_spd = speed
        
        # COMPUTE the duration (time in seconds) for which they should turn at these speeds.
        duration = length/speed
        
        return [duration, lft_whl_spd, rgt_whl_spd]
        
# =============================================================================
#     # Plan an Arc: accept input of arc radius (m), speed (m/s), and angle around the circle (RADIANS)
#     # ** NOTE the radius and angle both have Signs: Left turns are Positive, Right turns Negative
#     # Return [duration (s), left_wheel_speed (m/s), right_wheel_speed (m/s)]
# =============================================================================
    def plan_arc(self, radius, speed, angle):
        # COMPUTE the Left and Right wheel speeds using formulas from lecture
        lft_whl_spd = speed*(1 - (self.wheel_width/(2*radius) ) )
        rgt_whl_spd = speed*(1 + (self.wheel_width/(2*radius) ) )
        
        # COMPUTE "omega" - the angular rate of progress around the circle
        omega = speed/radius
        
        # Trick: correct funky combinations of signs
        omega = np.abs(omega)*np.sign(angle)
        
        # COMPUTE the duration (time in seconds) for which they should turn at these speeds.
        duration = angle/omega
        
        return [duration, lft_whl_spd, rgt_whl_spd]
        
# =============================================================================
#     # Plan a Pivot: accept angular velocity (rad/s) and angle to turn (rad)
#     # ** NOTE the angle and angular velocity both have signs by the Right Hand Rule: 
#     #    Counterclockwise: positive; Clockwise: negative
#     # Return [duration (s), left_wheel_speed (m/s), right_wheel_speed (m/s)]    
# =============================================================================
    def plan_pivot(self, omega, angle):
        # arguments: angular velocity (rad/sec), angle to turn (rad)

        # Trick: correct sign mismatches between omega and angle
        omega = np.abs(omega)*np.sign(angle)
        
        # COMPUTE wheel speeds using equations from lecture 
        radius = 0  # for a Pivot
        lft_whl_spd = omega*(radius-(self.wheel_width/2))
        rgt_whl_spd = omega*(radius+(self.wheel_width/2))
        
        # COMPUTE the duration (time in seconds) for which they should turn at these speeds.
        duration = angle/omega
        
        return [duration, lft_whl_spd, rgt_whl_spd]
        
    
# =============================================================================
#     # One more: allow the robot to sit still for a specified time (s)
#     # Return [duration (s), left_wheel_speed (m/s), right_wheel_speed (m/s)]      
# =============================================================================
    def plan_pause(self, pause_time): 
        lft_whl_spd = 0.
        rgt_whl_spd = 0.
        duration = pause_time
        
        return [duration, lft_whl_spd, rgt_whl_spd]        
####    CODE END    
    
