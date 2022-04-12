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
    
    
###############################################################################
# Functions that specify paths for closed-loop following
# using [x0, y0, theta0, Radius, Length] parameters. 
###############################################################################
    def specify_line(self, x0,y0,xf,yf):
        ### Function to create specs for a line from [x0,y0] to [xf,yf]. 
        # arguments: x0 (m), y0 (m), xfinal (m), yfinal (0)
        ## Turn them all into Floats (Python 2 requires this to avoid integer math)
        x0 = float(x0)
        y0 = float(y0)
        xf = float(xf)
        yf = float(yf)
        
####    CODE HERE: 
        vec = np.array([0.,0.]) # vector from [x0,y0] to [xf,yf], stored as a numpy array (np.array())
        dist = 0.0     # distance from [x0,y0] to [xf,yf], a Scalar
        vecangle = 0.0       # Vector angle: angle of the vector directly to the end point. A Scalar, compute with numpy's "arctan2"
####    CODE END
        return [x0, y0, vecangle, np.inf, dist]    # Return


    def specify_arc(self, x0,y0,xf,yf,R,way='short'):
        ### Function to create specs for an arc from [x0,y0] to [xf,yf] with radius R, 
        ###   going the "short way" or the "long way" around the circle.
        # arguments: x0 (m), y0 (m), xfinal (m), yfinal (0), Radius (m), way (='short' or 'long')
        ## Turn them all into Floats (Python 2 requires this to avoid integer math)
        x0 = float(x0)
        y0 = float(y0)
        xf = float(xf)
        yf = float(yf)
        R = float(R)

####    CODE HERE:  Edit everything that has 0.0 or [] assigned to give it the right values!       
        vec = np.array([0.0,0.0]) # vector from [x0,y0] to [xf,yf], stored as a numpy array (np.array())
        dist = 0.0     # distance from [x0,y0] to [xf,yf], a Scalar
        
        # Code to determine how a circular path of Radius R can get from start to finish of this path segment.  
        if dist > 2.0*np.abs(R):            # If the endpoint is farther away than the diameter of the circle, an arc cannot get there and you must draw a line instead. 
            specs = []  # Use the "self.specify_line(...)" function above for this case! 
        else:                       # Otherwise find an arc. 
            arcangle = 2.0*np.arcsin((dist/2.0)/R )     # Smallest arc angle is swept if going the "short way" around the circle. The distance from start to finish is the "chord length" and relates to the included angle as shown. 
            if way.lower() == 'long':
                arcangle = 2*np.pi*np.sign(arcangle) - arcangle     # arc angle swept if going the "long way": a full circle is 2*pi, so the long way is 2*pi minus the short way. Note the signs. 
            vecangle = 0.0       # Vector angle: angle of the vector directly from the start point to the end point.
            initangle = vecangle - arcangle/2.0         # Initial tangent angle of the arc to be followed. Deviates from the direct straignt line by half the included angle of the arc. 
            initangle = fix_angle_pi_to_neg_pi(initangle)   # Could have gone past pi with the addition. Unwrap to make sure it's within +/- pi. 
            arclength = 0.0                      # arc length is also needed. Enter it based on Radius and Angle. 
            specs = [x0,y0,initangle, R, arclength]
####    CODE END        
        
        return specs	
    
		

		
#%% 
#############################################################
# General Utility Functions        
#############################################################
def fix_angle_pi_to_neg_pi(angle): 
    # Function to "unwrap" angles into the [-pi, pi) interval
    return np.mod(angle+np.pi,2*np.pi)-np.pi        


####    CODE HERE: 
def convert_stage_settings_to_path_specs(xytheta_init, stages, wheel_width):
    ### Kinematic utility function to convert "stage settings" commands 
    ### (time, left_wheel_speed, right_wheel_speed)
    ### Into Path Geometry (series of arcs).
    
    # Initial point is as given: 
    xytheta_beginning = xytheta_init
    
    # Create an empty array to hold the path specs (variable "paths")
    paths = np.array([[]])    
    
    # Loop through the "stages" (like "stage_settings" from past versions) and convert each to a set of "path_specs" 
    for ii in range(0,stages.shape[0]):
        # First unpack the stage_settings
        xy_beg = xytheta_beginning[0:2]
        theta_beg = xytheta_beginning[2]    # What is the initial angle? 
        delta_time = stages[ii,0]    # Duration of the stage
        left_vel = stages[ii,1]    # Left wheel velocity
        right_vel = stages[ii,2]    # Right wheel velocity
        
####    CODE HERE: 
        # Then compute the robot's kinematic state during this Stage
        robot_vel = 0.0    # Determine the velocity of the robot
        robot_omega = 0.0    # Determine the angular velocity of the robot
        # Then compute the length and angle of the path it follows
        path_length = 0.0    # Determine the path segment's length (e.g. use speed and time)
        delta_theta = 0.0    # Determine the path segment's change in angle 

        # Determine the radius of curvature of the path segment
        if robot_omega == 0:    
            path_radius = 0.0    # If there's Zero angular velocity, what does that mean about the Radius? 
        else:
            path_radius = 0.0    # Otherwise what's the standard relationship among Radius, velocity and angular velocity? 
####    CODE END     
        
        ## Now compute where the path segment will end    
        # First, the ending tangent angle of the path
        theta_end = theta_beg+delta_theta 

        # Next the endpoint location in space 
        if np.isinf(path_radius):    # First handle the case of a Line
            xy_end = xy_beg + path_length*np.array([-np.sin(theta_beg), np.cos(theta_beg)])
        else:        # Otherwise handle Arcs. 
            # find circle center. Note it is a distance of (-R) out the wheel axis to the Right of the robot. 
            circle_center = xytheta_beginning[0:2] + -path_radius*np.array([np.cos(theta_beg), np.sin(theta_beg)])
            xy_end = circle_center + path_radius*np.array([np.cos(theta_end), np.sin(theta_end)])
        
        # Put the new path into a list of sequential path segments. 
        # Includes error handling for the case when there's only one - to make sure the result is the right shape. 
        if not (xy_end == xy_beg).all() :
            if not paths.any():
                paths = np.atleast_2d(np.append(xytheta_beginning,[path_radius,path_length]))
            else:
                paths = np.vstack((paths,np.append(xytheta_beginning,[path_radius,path_length])))
        
        # Store the new endpoint and end angle as the beginning of the next one. "append" just puts these together (not appending them to any other variable).
        xytheta_beginning = np.append(xy_end,theta_end)
        
    return paths				

