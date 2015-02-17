#!/usr/bin/env python
import roslib; roslib.load_manifest('vision')
import rospy


#rospy.set_param('/ABBparams', [1., 2., 3., 4.,5.0,6.0,7.0])

workobject = rospy.get_param('/ABB/workobject')
print workobject

#rospy.set_param('/ABBparams', [1.11, 222.23, 3.2323, 4.23,5.54650,6.22,7446.70])
#test = rospy.get_param('/ABBparams')
print test
