#!/usr/bin/env python
import roslib; roslib.load_manifest('vision')
import roslib; roslib.load_manifest('robot_comm')
import sys
import rospy
import cv
import cv2
import math
import numpy as np
import time

from vision.srv import start
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from robot_comm.srv import robot_SetJoints

class background():

    def init(self):
        xx=0

    def callback(self,image):       
        bridge = CvBridge() #make an object from the CvBridge class
        try:
            
            self.bat = bridge.imgmsg_to_cv(image, "bgr8") #convert from a ROS iamge to cv image
        except CvBridgeError, e:
            print e  # if it fails the, make error message
        
        
        
    def set_robojoints(self,pos):
        rospy.wait_for_service('/robot_SetJoints')
        try:
            robotjoints = rospy.ServiceProxy('/robot_SetJoints', robot_SetJoints)
            resptool = robotjoints(pos[0],pos[1],pos[2],pos[3],pos[4],pos[5])     
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        rospy.wait_for_service('/robot_SetJoints')

    def run(self,temp):
        
    
        background.set_robojoints((90.0, 0.0, 0.0, 0.0, 90.0, 0.0))
        rospy.Subscriber("/camera/rgb/image_color", Image, background.callback) #subscribe to the camera topic, and start the function callback    
        time.sleep(6)
        cv.SaveImage("/home/simplehands/Documents/hands/code/nodes/ABBnodes/vision/batt/background.jpg", self.bat)
        return(1,2,3,4,5,6,7)    
           
    def main(self):
        
        s = rospy.Service('abbgetbackgroundimage', start, background.run)

        
        rospy.spin() #keps the node running
   

if __name__ == '__main__':
    
    background=background()
    rospy.init_node('getbackground')     #init the node
    background.init()
    background.main()

