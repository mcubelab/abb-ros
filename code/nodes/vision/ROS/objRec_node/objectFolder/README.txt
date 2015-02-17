##########################################################################
# Object Recognition Node
#
# Object Folder Description
#
##########################################################################
#
# The MLab object recognition system recognizes objects from a library of
# known objects. This folder provides everything needed for the system to
# understand the object library.
#
# In order for the vision system to work, it requires 3 main things: 
# 1) The model of the object in point cloud form
# 2) A way to get an initial guess about where the object is in a point
# cloud
# 3) (Optional) Any symmetries associated with the object
# 
# The following folders and files contained within them allow the vision
# system to access this information and properly recognize objects.
#
#
# Folder descriptions:
# 
# object_descriptions: This folder contains all of the *.objd files which
# describe everything we need to know about the objects we are trying to
# recognize, including how to get an initial guess about its position, a
# link to its point cloud file, and any symmetries it might have.
#
# pcd_files: This folder contains all of the point cloud files for the
# objects. All files referenced in the *.objd files will be in this folder.
#
# solidworks_files: Contains 3D solidworks drawings of the objects in our
# library. These are then saved as STL files, which breaks up the solid
# model into a triangular mesh
#
# stl_files: Contains the STL files for all of the objects in our library.
#
# scripts: Contains a script that converts STL files into PCD files. In this
# way, the user can go from a CAD model of their object, to an STL file, to
# a point cloud file. Also contains a matlab script that can help visualize
# and check a .objd file for consistency
#
#
