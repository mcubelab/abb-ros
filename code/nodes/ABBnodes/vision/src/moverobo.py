#!/usr/bin/env python
import roslib
roslib.load_manifest('vision')
roslib.load_manifest('robot_comm')

import cv,time,sys,rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from robot_comm.msg import robot_CartesianLog
from robot_comm.srv import robot_SetCartesian

#rospy.init_node('moverobo') 

def add_two_ints_client():
    print "1"
    rospy.wait_for_service('robot_SetCartesian')
    print "2"
    try:
        robot_SetCartesian1 = rospy.ServiceProxy('/robot_SetCartesian', robot_SetCartesian)
        resp1 = robot_SetCartesian1(612.2, 360.7, 540.7, 0.0, -0.706, -0.708, 0.0)
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    add_two_ints_client()




