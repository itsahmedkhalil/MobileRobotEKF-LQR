#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Feb 12 22:27:44 2022

@author: el big boss
"""
import numpy as np
from matplotlib import pyplot as plt


leftEncoder = [15579, -18850, 13384, -14534, 11492, -7864, 11002, 1417, 10897]
rightEncoder = [0, 34989, -1374, 26302, -2584, 16549, -3028, 6308, -3052]

leftForward = []
leftBackward = []
rightForward = []
rightBackward = []

for i in range(len(leftEncoder)-1):
    if (i % 2) == 0:
        rate = (leftEncoder[i+1] - leftEncoder[i])/5     
        leftBackward.append(rate) 
        print(rate)
    else:        
        rate = (leftEncoder[i+1] - leftEncoder[i])/5     
        leftForward.append(rate) 
        print(rate)


for i in range(len(rightEncoder)-1):
    if (i % 2) == 0:
        rate = (rightEncoder[i+1] - rightEncoder[i])/5     
        rightBackward.append(rate) 

    else:
        rate = (rightEncoder[i+1] - rightEncoder[i])/5     
        rightForward.append(rate) 

####CAREFUL CAREFUL CAREFUL PAY ATTENTION
####RIGHT FORWARD AND BACKWARD COULD BE SWAPPED DEPENDING ON HOW YOU DEALT WITH THE NEGATIVE SIGN WITH ENCODER VALUES

print(leftForward, leftBackward, rightForward, rightBackward)
leftForwardGain = (leftForward[3] - leftForward[0])/360
rightForwardGain = (rightForward[3] - rightForward[0])/360
leftBackwardGain = (leftBackward[3] - leftBackward[0])/360
rightBackwardGain = (rightBackward[3] - rightBackward[0])/360
print(leftForwardGain, leftBackwardGain, rightForwardGain, rightBackwardGain)


# 
t = np.array([120, 240, 360, 480])
# ax1 = plt.subplot(211)
# ax1.set_title('Right Forward')
# ax1.set_ylabel('Encoder Rate')
# ax1.set_xlabel('Command Speed')
# plt.plot(t, rightForward)
# plt.tick_params('x', labelsize=6)

# ax2 = plt.subplot(212, sharex=ax1)
# ax2.set_title('Right Backward')
# ax2.set_ylabel('Encoder Rate')
# ax2.set_xlabel('Command Speed')
# plt.plot(t, rightBackward)
# # make these tick labels invisible
# plt.tick_params('x', labelbottom=False)

# ax3 = plt.subplot(221)
# ax3.set_title('Left Forward')
# ax3.set_ylabel('Encoder Rate')
# ax3.set_xlabel('Command Speed')
# plt.plot(t, leftForward)
# plt.tick_params('x', labelsize=6)

# ax4 = plt.subplot(222, sharex=ax3)
# ax4.set_title('Left Backward')
# ax4.set_ylabel('Encoder Rate')
# ax4.set_xlabel('Command Speed')
# plt.plot(t, leftBackward)
# # make these tick labels invisible
# plt.tick_params('x', labelbottom=False)

# plt.show()

fig, axs = plt.subplots(2, 2)
axs[0, 0].plot(t, rightForward)
axs[0, 0].set_title('Right Forward')
axs[0, 0].set_ylabel('Encoder Rate')
axs[0, 0].set_xlabel('Command Speed')
axs[1, 0].plot(t, rightBackward)
axs[1, 0].set_title('Right Backward')
axs[1, 0].set_ylabel('Encoder Rate')
axs[1, 0].set_xlabel('Command Speed')
axs[0, 1].plot(t, leftForward)
axs[0, 1].set_title('Left Forward')
axs[0, 1].set_ylabel('Encoder Rate')
axs[0, 1].set_xlabel('Command Speed')
axs[1, 1].plot(t, leftBackward)
axs[1, 1].set_title('Left Backward')
axs[1, 1].set_ylabel('Encoder Rate')
axs[1, 1].set_xlabel('Command Speed')

fig.tight_layout()
plt.show()