#!/usr/bin/python
#
# fing_calc.py
#
# Last Modified: 2/27/2014
#
# Author: Robbie Paolini
#
# A simple python script to compute the offsets and widths of boxes which
# encompass the finger. Known dimensions are specified as constants at the
# top of the program, and these are converted into homogeneous transforms
# and box widths
#
# The finger frame is the following:
# x - straight up, perpendicular to the palm
# y - straight towards the center of the palm
# z - Axis around which the finger rotates
#
# Currently, the script is set up with the following assumptions:
# 1) There are 2 rectangular prisms both whose center plane is coincident
# with the x-y plane.
# 2) The first rectangular prism has no rotation, and is merely offset from
# the origin of the finger
# 3) The second rectangular prism can be tilted at a given angle in the
# counter-clockwise direction about z, and its corner with y value closest
# to 0 is coincident with the first block's corner with greatest x value
# and y value closest to 0




import math

# CONSTANTS
# The point where one of each rectangle's corner's meet (x, y)
CORNER_PT = [36.11, -6.685]
# The full size of block 1 (x, y, z)
BLOCK_SIZE_1 = [50.0, 18.0, 20.0]
# The full size of block 2 (x, y, z)
#BLOCK_SIZE_2 = [55.6902, 25.0, 20.0]
BLOCK_SIZE_2 = [55.6902, 30.0, 25.0]
# The angle block 2 is rotated, in the counter clockwise direction
ANGLE = 24.0 / 180.0 * math.pi


#
# COMPUTATION
#

# Compute the center of block 1:

center_1 = [0.0, 0.0, 0.0]

center_1[0] = CORNER_PT[0] - BLOCK_SIZE_1[0] / 2.0
center_1[1] = CORNER_PT[1] - BLOCK_SIZE_1[1] / 2.0

# The orientation of block 1 is assumed to be in line

quat_1 = [1.0, 0.0, 0.0, 0.0]

# The width of block 1 is simply half the size in each dimension
width_1 = BLOCK_SIZE_1
for i in range(3):
  width_1[i] = width_1[i] / 2.0

# Compute the center of block 2
center_2 = [0.0, 0.0, 0.0]

# We find the center based on the angle that the block makes with the first
# block, and keeping in mind that their corners intersect
center_2[0] = CORNER_PT[0] + BLOCK_SIZE_2[0] / 2.0 * math.cos(ANGLE) - BLOCK_SIZE_2[1] / 2.0 * math.sin(ANGLE)
center_2[1] = CORNER_PT[1] - BLOCK_SIZE_2[0] / 2.0 * math.sin(ANGLE) - BLOCK_SIZE_2[1] / 2.0 * math.cos(ANGLE)

# Compute the orientation of block 2, which represents a rotation around z
quat_2 = [math.cos(-ANGLE/2), 0.0, 0.0, math.sin(-ANGLE/2)]

# The width of block 2 is simply half the size in each dimension
width_2 = BLOCK_SIZE_2
for i in range(3):
  width_2[i] = width_2[i] / 2.0

# Now print out poses and widths of each box
print "fingerBoxOffsets[0]"
print '[{:3.2f}, {:3.2f}, {:3.2f}, {:3.4f}, {:3.4f}, {:3.4f}, {:3.4f}]'.format(center_1[0], center_1[1], center_1[2], quat_1[0], quat_1[1], quat_1[2], quat_1[3])

print "fingerBoxOffsets[1]"
print '[{:3.2f}, {:3.2f}, {:3.2f}, {:3.4f}, {:3.4f}, {:3.4f}, {:3.4f}]'.format(center_2[0], center_2[1], center_2[2], quat_2[0], quat_2[1], quat_2[2], quat_2[3])


print "fingerBoxWidths[0]"
print '[{:3.2f}, {:3.2f}, {:3.2f}]'.format(width_1[0], width_1[1], width_1[2])
print "fingerBoxWidths[1]"
print '[{:3.2f}, {:3.2f}, {:3.2f}]'.format(width_2[0], width_2[1], width_2[2])
