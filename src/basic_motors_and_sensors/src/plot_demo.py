#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Quick demo of graphing data
"""

import numpy as np
from matplotlib import pyplot as plt

# Enter some data
xdata = np.array([-7, -3,-1,0,2,4,8])
ydata = np.array([-10,-5,0,13,27,44,76])

# Use Matplotlib/Pyplot to plot it. 
# Note, many aspects are a lot like MATLAB plotting. 
fig = plt.figure()  # create a Figure
p = plt.plot(xdata,ydata,'o-r')  # add a plot to it: first xdata, then ydata. "o-r" means "circle markers, with a line, red"
plt.xlabel("X Label Here")
plt.ylabel("Y Label Here")
plt.title("Title Here")
#plt.show()  # Sometimes this is necessary to show the plot. But often not. 

#If that shows up in the console at right (instead of as a separate window), then go to Tools->Preferences->IPython console-> Graphics->Graphics Backend and select "Automatic".
print('done')