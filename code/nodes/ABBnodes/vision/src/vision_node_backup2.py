#!/usr/bin/env python
import roslib
roslib.load_manifest('vision')
import sys
import rospy
import cv
import cv2
import numpy as np


from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class vision():

    def init(self):
        self.x = 0
        #print "5"
        
    def callback(self,image):       
        bridge = CvBridge() #make an object from the CvBridge class
        try:
            #print "6"
            self.cv_image = bridge.imgmsg_to_cv(image, "bgr8") #convert from a ROS iamge to cv image
        except CvBridgeError, e:
            print e  # if it fails the, make error message   
        
        
        cv.ShowImage("Image window", self.cv_image) #show image in the window
        cv.WaitKey(3)       # Think it's the update freakens of the window.
        #print "Before save"
        if (self.x == 0):
            pic = cv.SaveImage("image1.jpg", self.cv_image)
            print "saving image, as image1.jpg"
            print type(self.cv_image)
            print cv.GetDims(self.cv_image), cv.CV_MAT_CN(cv.GetElemType(self.cv_image))
            
            self.x+=1


    


#    def ImageManipulation(self):
        #print "7"
        #imagecv2 =cv2.imread('image1.jpg')
        #image3 = cv.GetImage(self.cv_image)
        #image=cv.LoadImage('image1.jpg', cv.CV_LOAD_IMAGE_COLOR) #Load the image
        #image=self.cv_image        
        #print "---"        
        #print type(vision.cv_image)
        #print cv.GetDims(self.cv_image), cv.CV_MAT_CN(cv.GetElemType(self.cv_image))
        #print type(self.cv_image)        
        image = cv.GetImage(vision.cv_image)        
        #print type(image)
        #print cv.GetDims(image), cv.CV_MAT_CN(cv.GetElemType(image))
        #print type(imagecv2)
        #print type(image3)
        temp=cv.CloneImage(image)           
        #tem2p=cv.CloneImage(self.cv_image)
        '''ROI'''
        x= 465 # co-ordinate of top left vertex.
        y= 254 # co-ordinate of top left vertex.
        w= 34 # width or region
        h= 28 # height of region
        cv.SetImageROI(image,(x,y,w,h))
        
        #print "8"
        picsize = cv.GetSize(image) #get the size of image
        
        '''Convert to grey'''
        w=picsize[0]    
        h=picsize[1]
        no_of_bits=8
        channels=1
        grey=cv.CreateImage((w,h),no_of_bits,channels) #Make a grey image
        cv.CvtColor(image,grey,cv.CV_BGR2GRAY)  #convert the color image and save it in the new gray image
      
        """Make binary image"""
        threshold=150
        colour=255
        cv.Threshold(grey,grey, threshold,colour,cv.CV_THRESH_BINARY)
        
        #cv.ShowImage("Loaded image window", grey) #show image in the window
        cv.WaitKey(3)       # Think it's the update freakens of the window. It is!
        
        """Remove Error blobs"""
        cv.Erode(grey,grey,None,1)
        cv.Dilate(grey,grey,None,1)
        #cv.ShowImage("Erodede / dilatede Image", grey)

        binimage=cv.CloneImage(grey)
        cv.Threshold( grey, binimage, 200, 255, cv.CV_THRESH_BINARY )
        cv.ShowImage("binimage Image", binimage)

        """Find all contours."""
        stor = cv.CreateMemStorage()
        print stor
        image04 = None
        cv.Zero(image04)    
        binimagematrix=vision.cv2array(binimage)
        cont, hierarchy = cv2.findContours(binimagematrix,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
        #largest_contour = cont  
        print 'length of cont %s' % len(cont)
        print cont
        
        

#        for c in largest_contour:
#            cv.Circle(temp, c, 1, (255,255,0), 8)
        cnt = cont[0]
        moments = cv2.moments(cnt)
        (x,y),(MA,ma),angle = cv2.fitEllipse(cnt) 
        (xRect,yRect),(wRect,hRect),thetaRect = cv2.minAreaRect(cnt)
        print x
        print y
        print angle
        print " "
        print xRect
        print yRect
        print wRect
        print hRect       
        print thetaRect  
        
        length = 10
        x1 = int(xRect)
        y1 = int(yRect)
        x2 = int(xRect + np.sin(thetaRect)*length)
        y2 = int(yRect + np.cos(thetaRect)*length)

        binimage_rgb = cv.CreateMat(binimage.height, binimage.width, cv.CV_8UC3)
        cv.CvtColor(binimage,binimage_rgb,cv.CV_GRAY2RGB)
            
        cv.Line(binimage_rgb,(x1,y1),(x2,y2),(0,255,0),2,8)
        cv.ShowImage('original image', binimage_rgb) 


    def cv2array(self,im):
        depth2dtype = {
            cv.IPL_DEPTH_8U: 'uint8',
            cv.IPL_DEPTH_8S: 'int8',
            cv.IPL_DEPTH_16U: 'uint16',
            cv.IPL_DEPTH_16S: 'int16',
            cv.IPL_DEPTH_32S: 'int32',
            cv.IPL_DEPTH_32F: 'float32',
            cv.IPL_DEPTH_64F: 'float64',
        }

        arrdtype=im.depth
        a = np.fromstring(
            im.tostring(),
            dtype=depth2dtype[im.depth],
            count=im.width*im.height*im.nChannels)
        a.shape = (im.height,im.width,im.nChannels)
        return a

 
def main():
    rospy.init_node('vision_node')     #init the node
    print "1"
    vision.init()
    print "2"
    cv.NamedWindow("Image window", 1) #start a window
    
    rospy.Subscriber("/camera/rgb/image_color", Image, vision.callback) #subscribe to the camera topic, and start the function callback
    print "3"
#    vision.ImageManipulation()
    print "4"
    
    #rospy.spin() #keps the node running

   

if __name__ == '__main__':
    print "0"
    vision=vision()
    main()

