#!/usr/bin/env python
import roslib; roslib.load_manifest('vision')
import sys
import rospy
import cv
import cv2
import time
import numpy as np

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

class image():

    def init(self):   
        self.xx = 0 

    def callbackHD(self,HDimage):       
        bridge = CvBridge() #make an object from the CvBridge class
        try:
            
            self.cv_imageHD = bridge.imgmsg_to_cv(HDimage, "bgr8") #convert from a ROS iamge to cv image
        except CvBridgeError, e:
            print e  # if it fails the, make error message   

        
        key=-1
        key=cv.WaitKey(10)
        if (key != -1):
            pic = cv.SaveImage("HD/HDtest"+str(self.xx)+".jpg", self.cv_image)
            self.xx =self.xx+1
       
   
            
        
        #image2 = cv.GetImage(self.cv_image)
        size=cv.GetSize(self.cv_image)
        print size
        cv.Line(self.cv_image,(0,0),size,(255,0,255),1,8)
        cv.Line(self.cv_image,(640,0),(0,480),(255,0,255),1,8) 
  
        cv.ShowImage("image", self.cv_image)
        cv.WaitKey(3)


    def main(self):
        image.init()
        cv.NamedWindow("Image window", cv.CV_WINDOW_AUTOSIZE) #start a window  
        rospy.init_node('HDpics')     #init the node
        rospy.Subscriber("/vis_hand/camera/image_raw", Image, image.callbackHD) #subscribe to the camera topic, and start the function callback
        time.sleep(3)
        image2 = cv.GetImage(self.cv_image)
        cv.ShowImage("image2", image2)
        rospy.spin() 

   

if __name__ == '__main__':
    
    image=image()
    image.main()
