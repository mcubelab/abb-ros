#!/usr/bin/env python
# 
# Just receive the print the force data from the ATI mini-40 force sensor
# which we will be using for calibration.

import sys
import math
import numpy

import roslib; roslib.load_manifest('garthzScripts')
import rospy

# from netft_rdt_driver.msg import *
from geometry_msgs.msg import *

################################################################
# receive the force sensor topic data
def force_sensor_update(data):
    print "force Z:", data.wrench.force.z

################################################################
def main():
    # initialize ROS
    rospy.init_node('echo_ATI_force_sensor')

    # set up force data callback
    rospy.Subscriber("/netft_data", WrenchStamped, force_sensor_update )

    # wait for activity
    print "waiting for force data"
    while not rospy.is_shutdown():
        rospy.sleep( 0.1 )

if __name__ == "__main__":
    main()
