#!/usr/bin/env python
import roslib; roslib.load_manifest('vision')
import sys
import rospy
import cv
import cv2
import numpy as np

from vision.msg import posmsg

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

class vision():

    def init(self):
        self.xx = 0

        
        
    def callback(self,image):       
        bridge = CvBridge() #make an object from the CvBridge class
        try:
            
            self.cv_image = bridge.imgmsg_to_cv(image, "bgr8") #convert from a ROS iamge to cv image
        except CvBridgeError, e:
            print e  # if it fails the, make error message   

        
        if (self.xx == 0):
            pic = cv.SaveImage("test.jpg", self.cv_image)
            print "saving image, as test.jpg"
            print type(self.cv_image)
            print cv.GetDims(self.cv_image), cv.CV_MAT_CN(cv.GetElemType(self.cv_image))
            
            self.xx+=1
        
        image = cv.GetImage(vision.cv_image)  
        cv.ShowImage("image", image)
        cv.WaitKey(3)

def main():
    
    
    vision.init()
    cv.NamedWindow("Image window", cv.CV_WINDOW_AUTOSIZE) #start a window  
    rospy.init_node('vision_node')     #init the node
    rospy.Subscriber("/camera/rgb/image_color", Image, vision.callback) #subscribe to the camera topic, and start the function callback
    
   
    rospy.spin() 

   

if __name__ == '__main__':
    
    vision=vision()
    main()
