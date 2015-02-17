#!/usr/bin/env python
import roslib; roslib.load_manifest('vision')
import sys
import rospy
import cv
import cv2
import numpy as np
import time

from vision.srv import startt
from vision.msg import posmsg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class take():
    def init(self):
        rospy.Subscriber("/camera/rgb/image_color", Image, vision.callback)


    def callback(self,image):       
        bridge = CvBridge() #make an object from the CvBridge class
        try:
            self.cv_image = bridge.imgmsg_to_cv(image, "bgr8") #convert from a ROS iamge to cv image
        except CvBridgeError, e:
            print e  # if it fails the, make error message  
        time.sleep(6)
             
    def init(self):
        rospy.Subscriber("/camera/rgb/image_color", Image, take.callback)
        test=12
    
    def run(self,path):
        cv.SaveImage(str(path.a)+str(path.b)+".jpg", self.cv_image)
        print 'this is path:',path
        
        return(1,2,3,4,5,6,7)
        
    def main(self):
        
        rospy.Service('takeimage', startt, take.run)
        #cv.SaveImage("TESTTTTESTES.jpg", self.cv_image)
        temp=90        
        rospy.spin()

if __name__ == '__main__':
    take=take()
    take.init()
    rospy.init_node('takeimage')     #init the node
    take.main()
    
