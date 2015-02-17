#!/usr/bin/env python

# Import required libraries
import sys
import os
import string
from datetime import datetime
from collections import namedtuple
import math
import random

##################################################################
# Constant Declarations

# Scale to multiply STL file by to convert it into meters (pcl)
SCALE = 0.001

# Default number of points to sample on the object
DEFAULT_POINTS = 2000


##################################################################
# Function to refine stl file into points
#
# We uniformly sample the surface area, and generate as many points as 
# desired. This can be done by getting a list of all of the triangles 
# that make up the mesh, randomly picking a triangle based on its area, 
# and then randomly sampling within that triangle.
#
# Inputs: path to stl file, number of points to sample on the surface
# Output: list of points randomly generated on surface

def refineSTLtoPoints(filename):

  # Load our STL file
  stlfile = open(filename);
  
  # This will hold our list of triangles
  Surface = namedtuple('Surface', ['normal', 'vertices'])

  surfaces = []

  num_surfaces = 0

  found_surface = 0
  num_vertices = 0

  # This will keep track of the area of each triangle in the 
  # mesh, and the total area
  areas = []
  cumsum = 0.0

  # Go through the STL file, and pick out the normals and vertices
  NORMAL_LINE = "facet normal "
  VERTEX_LINE = "vertex "
  for line in stlfile:
    # If we have not yet found a surface, look for one
    if (found_surface == 0):
      s = string.find(line, NORMAL_LINE)
      if (s != -1):
        # If we have found a surface, initialize it, and 
        #  save the normal direction
        found_surface = 1
        num_vertices = 0
        cur_surface = Surface([0]*3, [[0]*3,[0]*3,[0]*3])
        
        s = string.find(line, NORMAL_LINE)
        nums = line[s+len(NORMAL_LINE):].split(" ")
        for i in range(3):
          cur_surface.normal[i] = float(nums[i])
    else:
      # If we have found a surface, now we need to read in our vertices
      s = string.find(line, VERTEX_LINE)
      if (s != -1):
        # If we found a vertex, read it in, and increment the number of 
        # vertices for this surface
        s = string.find(line, VERTEX_LINE)
        nums = line[s+len(VERTEX_LINE):].split(" ")
        for i in range(3):
          cur_surface.vertices[num_vertices][i] = float(nums[i])
        num_vertices = num_vertices + 1
        
        # If we have found all 3 vertices, now let's compute the area
        if (num_vertices == 3):
          # First, find 2 vectors that define the triangle
          v1 = [0.0]*3
          v2 = [0.0]*3
          for i in range(3):
            v1[i] = cur_surface.vertices[1][i] - cur_surface.vertices[0][i]
            v2[i] = cur_surface.vertices[2][i] - cur_surface.vertices[0][i]
          
          # Now, take the cross product to get a notion of the area
          cross = [0.0]*3
          cross[0] = v1[1]*v2[2]-v1[2]*v2[1]
          cross[1] = -v1[0]*v2[2]+v1[2]*v2[0]
          cross[2] = v1[0]*v2[1]-v1[1]*v2[0]
          
          # The sqrt(||cross(u,v)||)/2 gives us the area of the triangle
          mag = 0.0
          for i in range(3):
            mag = mag + cross[i]**2
          area = math.sqrt(mag)/2
          
          # Keep track of the total area, and save our running total 
          # for this surface
          cumsum = cumsum + area
          areas.append(cumsum)
          
          # Also save the surface
          surfaces.append(cur_surface)
          
          # Now we're done. Time to look for another surface
          found_surface = 0
          num_vertices = 0
  stlfile.close()


  # Now that we've found all of the triangles, normalize the sum of 
  #  all of the surface area to be 1
  for i in range(len(areas)):
    areas[i] = areas[i] / cumsum


  # now annies commenting, so.... WHATUP MOTHERFUCKERS
  # keep all the vertices, removing duplicates
  # for the block objects that should leave only the real vertices
  # also storing edges to find faces later
  # shitshitshitshitshitshitshitFUCKshitshit

  # total_verts = [[]]
  # for i in range(len(surfaces)):    
  #   for v in surfaces[i].vertices:
  #     tmp = []
  #     for p in v:
  #       tmp.append(round(p))
  #     if tmp not in total_verts:
  #       total_verts.append(tmp)
  #  return total_verts

  total_verts = [[]]
  for i in range(len(surfaces)):
    sur = []
    for v in surfaces[i].vertices:
      # fucking shit
      tmp = []
      for p in v:
        # that one was just for fun
        tmp.append(round(p))
      sur.append(tmp)
    total_verts.append(sur)
    
  return total_verts


###############################################################
###############################################################
# MAIN LOOP BEGINS HERE

# Get the directory of this python script
binfolder = os.path.dirname(os.path.realpath(__file__))

# First check that we have the right number of arguments
if (len(sys.argv) != 3 and len(sys.argv) != 4):
  print "usage: python stl2vert.py stlfile textfile"
  sys.exit(1)

# # Check to see how many points the user wants to generate for this object
# if (len(sys.argv) == 4):
#   num_points = int(sys.argv[3])
# else:
#   # Otherwise, generate our default number of points
#num_points = DEFAULT_POINTS

# Call our function to get a bunch of points on the surface of the 
#  object defined by the STL file
#points = refineSTLtoPoints(sys.argv[1], num_points)
verts = refineSTLtoPoints(sys.argv[1])
print "len " +str(len(verts))

f = open(sys.argv[2], 'w')

# for v in verts:
#   s = ""
#   print "v " +str(v)
#   for p in v:
#     s += str(p)+","
#   s += "\n"
#   f.write(s)

for sur in verts:
  s = ""
  for v in sur:
    for p in v:
      s += str(p)+","
    s += "\n"
  s += "!\n"
  f.write(s)

f.close()



# # Now that we have our final list of points, let's make our pcd file
# num_pts = len(points)

# pcdfile = open(sys.argv[2], 'w');

# # First print out a commented out header
# pcdfile.write("# .PCD v.7 Point Cloud Data file format\n")
# pcdfile.write("# Note: File auto-generated by stl2pcd.py on " + 
#     str(datetime.now()) + "\n")
# pcdfile.write("#  Input: " + sys.argv[1] + 
#     ", output: " + sys.argv[2] + "\n")

# # Now print out the required header for a PCD file
# pcdfile.write("VERSION .7\n")
# pcdfile.write("FIELDS x y z\n")
# pcdfile.write("SIZE 4 4 4\n")
# pcdfile.write("TYPE F F F\n")
# pcdfile.write("COUNT 1 1 1\n")
# pcdfile.write("WIDTH " + str(num_pts) + "\n")
# pcdfile.write("HEIGHT 1\n")
# pcdfile.write("VIEWPOINT 0 0 0 1 0 0 0 0\n")
# pcdfile.write("POINTS " + str(num_pts) + "\n")
# pcdfile.write("DATA ascii\n")

# # Finally, print out a list of all of our points
# for i in range(num_pts):
#   for j in range(len(points[i])):
#     pcdfile.write(str(points[i][j] * SCALE) + " ")
#   pcdfile.write("\n")

# # We're done, so close the file and exit
# pcdfile.close()

# print "Saved " + str(num_pts) + " points to " + sys.argv[2]

# os.system("pcd_viewer " + sys.argv[2] + " -ax 0.1")


