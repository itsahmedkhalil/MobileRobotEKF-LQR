#!/usr/bin/env python3

# =============================================================================
# Peter G. Adamczyk 
# Updated 2021-02-27
# =============================================================================

import numpy as np
from matplotlib import pyplot as plt

# Default file to read
svg_file = 'SVGtest.svg'

# Rectangular workspace    
# how much space do you have for the robot to move in (meters)? Measure its arena!
dx_arena = 1.0
dy_arena = 1.0    

    
# All-inclusive function to get path_specs from an SVG file.    
def convert_svg_to_path_specs(svg_file, xlength=dx_arena, ylength=dy_arena):
    svg_coords = parse_svg_for_paths(svg_file)
    scaled_coords = scale_coords_to_arena(svg_coords, xlength, ylength)
    path_specs = convert_coords_to_path_specs(scaled_coords)
    
    return path_specs

# All-inclusive function to get waypoints from an SVG file.    
def convert_svg_to_waypoints(svg_file, xlength=dx_arena, ylength=dy_arena):
    svg_coords = parse_svg_for_paths(svg_file)
    scaled_coords = scale_coords_to_arena(svg_coords, xlength, ylength)
    
    return scaled_coords    

# Simple function to parse the line paths in an SVG file (marked "d") into a set of absolute spatial coordinates. 
def parse_svg_for_paths(svg_file):
    # Find lines with 'd="' at the beginning: 
    d_lines = []
    ignore = 0    # control parameter to ignore "<clipPath>" sections
    with open(svg_file,'r') as svgfile:
        for line in svgfile:
            l = line.strip()
            if l[0:3]=='d="' and not ignore:
                d_lines.append(line.strip().strip(' <d="/>'))
            elif l[0:9] == '<clipPath':
                ignore = 1
            elif l[0:11] == '</clipPath>':
                ignore = 0
            
    # parse such lines to separate out anything delimited by space
    svg_coords = np.ndarray((0,2)).astype(float)
    d_line_origin = np.array([[0.,0.]])
    current_point = np.array([[0.,0.]])
    #first_absolute = 1 
    for d_line in d_lines:
        chunks = d_line.split(' ')  # Split the string at spaces
#        print(chunks)
        # Step through the chunks and parse out coordinates
        ii = 0
        while ii < len(chunks) : 
            chunk = chunks[ii]
#            print(chunk)
            if chunk[0].isalpha() :  # If chunk is alphabetic, it's a setting telling us what's coming. 
                
                # Case of mode command determines absolute (upper) or relative (lower)
                if chunk.isupper():
                    absolute = 1
                else : 
                    absolute = 0
                
                # use the lowercase version to settle what mode we are in. 
                mode = chunk.lower()
                # Set the number of chunks to increment
                if mode == "m":
                    incr = 1
                    # first M or m is always absolute, but this is a separate setting from Mode. 
                    first_absolute = 1
                elif mode == "l":
                    incr = 1
                elif mode == "h":  # horizontal segment
                    incr = 1
                elif mode == "v":  # vertical segment
                    incr = 1
                elif mode == "c":  # skip the control points
                    incr = 3
                elif mode == "s":  # skip the control point
                    incr = 2
                elif mode == "z":  # closing "z" or "Z"
                    incr = 1
                    svg_coords = np.append(svg_coords, d_line_origin, axis=0)
                else:
                    incr = 1
                    
                ii += incr
                continue
            
            else: 
                xy = chunk.split(',')
                xy = [float(s) for s in xy]
                
                # Deal with horizontal and vertical lines. 
                # They have only one number, so we have to fill the other with 0.0. 
                if mode == "h" and len(xy)==1:
                    if absolute: 
                        xy = [xy[0],current_point[1]]
                    else:
                        xy = [xy[0],0.0]
                elif mode == "v" and len(xy)==1: 
                    if absolute: 
                        xy = [current_point[0],xy[0]]
                    else:
                        xy = [0.0,xy[0]]
                
                # Now either initiate or append the new coordinate. 
                if first_absolute:
                    svg_coords = np.append(svg_coords, [[xy[0],xy[1]]],axis=0)
                    # set first_absolute to zero (not anymore). 
                    first_absolute = 0
                    d_line_origin = np.array([[xy[0],xy[1]]])
                else:
                    if absolute:
                        svg_coords = np.append(svg_coords, [[xy[0],xy[1]]],axis=0)
                    else:
                        svg_coords = np.append(svg_coords, [current_point + np.array([xy[0],xy[1]])],axis=0)
                
                current_point = svg_coords[-1]
                
                if ii < len(chunks)-1:
                    if chunks[ii+1].isalpha():
                        incr = 1
                
                ii += incr
                
    return svg_coords

# Function to scale the coordinates from SVG to a specific size of workspace (rectangle, dimensions dx and dy)            
def scale_coords_to_arena(coords, dx, dy):
    xsvg = coords[:,0]
    ysvg = coords[:,1]  # Note that Y is Down in SVG!
    
    xmin = np.min(xsvg)
    xmax = np.max(xsvg)
    xrng = xmax - xmin
    xctr = xmin + xrng/2
    ymin = np.min(ysvg)
    ymax = np.max(ysvg)
    yrng = ymax - ymin
    yctr = ymin + yrng/2

    # Shift and scale. 
    k = min([dx/xrng, dy/yrng])
    yscaled = -1*(ysvg-ymax)*k   # Note that Y is Down in SVG!
    xscaled = (xsvg-xmin)*k
    
    scaled_coords = np.vstack((xscaled,yscaled)).transpose()
    
    return scaled_coords
    
    
# Function to save coordinates in a CSV file 
def save_coords_to_csv(svgfile, csv_coords):
    np.savetxt(svg_file[:-4] + '.csv', csv_coords,delimiter=',')
        

# Function to convert coordinates into a series of path_specs 
def convert_coords_to_path_specs(coords):
## path_specs are:  [Xorigin, Yorigin, Theta_init, Rsigned, Length]

    coords = np.append([[0.,0.]],coords,axis=0)  # start at 0
    coords = np.append(coords,[[0.,0.]],axis=0)  # end at 0
    
    path_specs = np.ndarray((0,5)).astype(float)
    for ii in range(1,len(coords)):
        xorigin = coords[ii-1][0]
        yorigin = coords[ii-1][1]
        displacement = coords[ii]-coords[ii-1]
        angle = np.arctan2(-displacement[0], displacement[1])   # Remember the angle is measured from the +y axis. 
        rsigned = np.inf
        length = np.linalg.norm(displacement)
        
        path_specs = np.append(path_specs, [[xorigin, yorigin, angle, rsigned, length]], axis=0)
    
    return path_specs
    
    
# default behavior if the program is called as a stand-alone. 
if __name__=='__main__':
    svg_coords = parse_svg_for_paths(svg_file)
    scaled_coords = scale_coords_to_arena(svg_coords, dx_arena, dy_arena)
    save_coords_to_csv(svg_file, scaled_coords)
    path_specs = convert_coords_to_path_specs(scaled_coords)
    
    print(path_specs)
    
    # Plot it if you want to: 
    fig = plt.figure(); 
    plt.plot(scaled_coords[:,0],scaled_coords[:,1],'o-')
    plt.axis('equal')
    plt.xlabel('x'); plt.ylabel('y');