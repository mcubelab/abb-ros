##########################################################################
##########################################################################
# Object Recognition Node
#
# .objd file description
#
# Author: Robbie Paolini
#
# Last Modified: 12/9/2013
#
# This file decribes how to format an object description file 
# (extension .objd). In a .objd file, we provide a link to the point
# cloud file for this object, describe any symmetries associated with the
# object, and set up how to initially guess the object pose. See the .objd
# files in this folder for more examples of the format. We detail the
# format below.
#
#
#
##########################################################################
##########################################################################
#
# [GENERAL FORMAT]
# All lines that start with '#', or only contain space characters ('\n', '
# ', '\t', etc will be ignored
#
# The first non-ignored line will be the string name of the object,
# whenever it is referenced
#
# The second non-ignored line will be the name of the point cloud file,
# including the .pcd extension. Note that this file should be located in the
# 'pcd_files' folder, and the actual name of the file should not contain
# any part of the path, but only the name of the file
#
# A line beginning with a '$' symbol indicates that the lines that follow
# contain symmetry information about the object. See the [SYMMETRY FORMAT]
# section for more details. No ending character is required.
#
# A line beginning with a '&' symbol indicates that the lines that follow
# contain information on the guess function we will use to get an initial
# guess about the object. See the [GUESS FUNCTION FORMAT] section for more
# details.
#
# A line beginning with a '!' symbol indicates the end of the file.
# Everything below it will be ignored.
#
# 
#
##########################################################################
#
# [SYMMETRY FORMAT]
#
# The symmetry format is as follows:
# $ N
# ..
# ..
# ..
#
# The first line contains the '$' character, followed by a space, followed
# by a number representing the type of symmetry the object has. The
# following lines detail the symmetry.
#
# Below, we detail each of the types of symmetries, and the format of the
# rest of the symmetry section
# 
#
# SYMMETRY_NONE: 0
# No symmetry on this object. Note that this is not explicitly required to
# be in the file. If no '$' is found, it will be assumed that the object
# has no symmetry. Only the line '$ 0' is required, all lines below will be
# assumed to be about other aspects
#
#
# SYMMETRY_ORDER: 1
# This is for rotational symmetries of a certain order. The order N
# represents the number of degrees 360/N to rotate the object to arrive at
# a symmetric position. 3D objects can have more than one "order" symmetry.
# Because of this, here is how the ordered symmetry section should look:
#
# $ 1
# 3
# 0 0 1 0.0145 0.0145 0.0145 4
# 0 1 0 0.0145 0.0145 0.0145 2
# -1 1 1 0.0145 0.0145 0.0145 3
#
# The first line introduces that we will be using ordered symmetry
# The second line is how many ordered symmetries there are
# The lines after that describe the symmetries. They are in the following
# format, with each number separated by spaces:
#
# nx ny nz x y z r
#
# nx ny nz: represents the direction of the axis of rotation in the 
# frame of the object. Note that this does not necessarily have to be a
# unit vector, as it will be normalized later
# x y z: represents a point that the axis goes through in the frame of the
# object. Should be in meters.
# r: The order of the symmetry. 
#
# In the above example, we see that this object (which happens to be a cube
# with 29mm sides) has 3 ordered symmetries, 1 with order 4, one with order
# 3, and one with order 2.
#
# An important thing to note with these symmetries is that the symmetry
# transforms will be applied in the order they are specified. While
# often symmetries commute, in the case of the cube, they do not, and hence
# the order they are written in turns out to be important.
#
#
# SYMMETRY_CYLINDER: 2
# This symmetry is the same as SYMMETRY_ORDER, except the order is
# infinite, that is, the object is a cylinder. In this case, the format 
# is the following:
#
# $ 2
# 0 0 1.0 0 0 0 1
#
# The first line introduces the fact that we have a cylindrical symmetry.
# The second line represents the axis of rotation of the cylinder, with the
# format being similar to the SYMMETRY_ORDER case:
#
# nx ny nz x y z c
#
# nx ny nz: Represents the direction of the axis of rotation in the frame
# of the object, and does not necessarily have to be a unit vector.
# x y z: Represents a point in the frame of the object that the axis goes
# through. Should be in meters.
# c: 1 if our object has a plane of symmetry going through it's center
# (like a cylinder), and 0 if not (like a cone). If c = 1, then x y z 
# must represent the center of our object in order for this second symmetry
# to be taken into account.
#
#
# SYMMETRY_SPHERE: 3
# Spherical symmetry. Use this if your object is a sphere. The format is
# very simple:
#
# $ 3
# 0.1 0.1 0.1
#
# The first line represents that we have spherical symmetry, and the second
# line is the center of the sphere in the frame of the object, with units
# in meters.
#
#
#
##########################################################################
#
# [GUESS FUNCTION FORMAT]
#
# The guess function format is as follows (similar to the symmetry format):
#
# & N
# ..
# ..
# ..
#
# The first line explains what guess function we will be using for this
# object, and the latter lines describe the guess function in more detail,
# if necessary. We describe each type of guess function and their required
# parameters below:
#
# GUESS_NONE: 0
# This is if you wish to use no guess function for this object. This is not
# recommended, and will almost certainly result in the vision system
# failing to recognize the object. Note that the line '& 0' is not
# explicitly required, as if the '&' symbol is not found, the system will
# be forced to assume that no guess function will be used.
#
#
# GUESS_ELLIPSOID: 1
# This guess function simply computes the centroid of both the point cloud
# seen by the camera and the point cloud of the object to get the
# translation, and then gets the rotation by computing the correlation
# matrix of both point clouds, and finding the rotation that causes the
# eigenvalues to match. In effect, what's happening is we're fitting an
# ellipsoid to the data, and matching the axes. The format for ellipsoid
# fitting is as follows:
# 
# & 1
# 2
#
# The first line indicates that we'll be using ellipsoid fitting, and the
# second line indicates if there are any symmetries for this object. 
# If the object is not symmetric, then, we'll have to check different 
# directions of major and minor axes of our ellipsoid. 
#
# The second line is a single number, 0, 1, or 2. 
# 0: There are no symmetries, and all 4 possible matches of major and 
# minor axes will be checked. 
# 1: The object looks like a cone, which means only 2 possible matches of 
# major and minor axes will be checked.
# 2: The object looks like a cylinder, which means only a single 
# combination of match needs to be checked.
# 
# Note that this method, while simple, is somewhat unstable in the case 
# of occlusions, so should be used with care.
#
#
# GUESS_PLANES: 2
# This guess function finds the planes on both object and point cloud, and
# matches them. For the best accuracy, the user provides both a description
# of the planes and a description of which planes form each vertex of the
# object. This guess function is especially useful for polygonal objects,
# and is much less susceptible to occlusion. The format is as follows:
#
# & 2
# [0 0 1 -0.029; 0 0 -1 0; 1 0 0 -0.029; -1 0 0 0; 0 1 0 -0.029; 0 -1 0 0]
# [1 3 5; 1 3 6; 1 4 5; 1 4 6; 2 3 5; 2 3 6; 2 4 5; 2 4 6]
# 
# The first line indicates that we will be using planes to guess the
# location of the object. The second line describes the planes contained by
# the object, and the third line indicates which planes intersect at points
# along the object. 
#
# The second line is in the following format:
# 
# [A1 B1 C1 D1; A2 B2 C2 D2; ...]
# 
# These represent the following equations of planes, again with respect to
# the frame of the object: Ax + By + Cz + D = 0. A critical thing to note
# is that [A B C] is the direction of the OUTWARD facing normal. If this is
# not followed, the guess function will fail. 
#
# The third line has the following format:
#
# [p11 p12 p13; p21 p22 p23; ...]
#
# 3 non-parallel planes intersect at a point, and each group of 3 numbers
# represent the indices of planes that intersect. Note that these numbers
# are 1-indexed. In the above example, we see that our cube has 6 faces,
# and has 8 intersections.
#
#
# GUESS_FEATURES: 3
# This guess function computes a Fast Point Feature Histogram (FPFH) of the
# object, and uses Sample Consensus Initial Alignment (SAC-IA), to find a
# good match. See "Fast Point Feature Histograms (FPFH) for 3D
# Registration" by Rusu, Blodow, and Beetz for more information. This 
# guess function is especially useful for objects that have a lot of 
# depth features, and will probably not work as well on smooth objects 
# like spheres. The format is as follows:
#
# & 3
# 0.02 0.01 10 0.00427 5
#
# The first line represents the fact that we want to use feature matching
# as our initial guess. The second line is all of the parameters we would
# like to use for our feature matching. These are explained below:
#
# [normal_radius] [feature_radius] [num_samples] [min_sample_dist] [k_corr]
#
# normal_radius: The radius of points to use (in meters) to estimate the
# outward facing normal of the surface of the object at that point
#
# feature radius: The radius of normals to use (in meters) when creating
# our feature histograms.
#
# num_samples: number of sample points to select when doing the matching.
#
# min_sample_dist: The minimum distance that 2 sample points can be
# separated from each other, in meters.
#
# k_corr: The number of points whose histograms that are similar to a
# sample point's histogram to select from when matching.
# 
#
# 
##########################################################################


