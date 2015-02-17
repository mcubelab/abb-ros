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

#posmsg.posx =0

#print posmsg.posx

class vision():

    def init(self):
        self.xx = 0

        
        
    def callback(self,image):       
        bridge = CvBridge() #make an object from the CvBridge class
        try:
            
            self.cv_image = bridge.imgmsg_to_cv(image, "bgr8") #convert from a ROS iamge to cv image
        except CvBridgeError, e:
            print e  # if it fails the, make error message   
        
        """
        cv.ShowImage("Image window", self.cv_image) #show image in the window
        cv.WaitKey(3)       # Think it's the update freakens of the window.
        """
        if (self.xx == 0):
            pic = cv.SaveImage("image14.jpg", self.cv_image)
            print "saving image, as image14.jpg"
            print type(self.cv_image)
            print cv.GetDims(self.cv_image), cv.CV_MAT_CN(cv.GetElemType(self.cv_image))
            
            self.xx+=1
        
        image = cv.GetImage(vision.cv_image)        
        
        temp=cv.CloneImage(image)           
        
        '''ROI'''
        xroi= 400 # co-ordinate of top left vertex.
        yroi= 160 # co-ordinate of top left vertex.
        wroi= 176 # width or region
        hroi= 121 # height of region
        cv.SetImageROI(image,(xroi,yroi,wroi,hroi))
        
        
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
        
        
        cv.WaitKey(3)       # Think it's the update freakens of the window. It is!
        
        """Remove Error blobs"""
        #cv.Erode(grey,grey,None,1)
       #cv.Dilate(grey,grey,None,1)
        

        binimage=cv.CloneImage(grey)
        cv.Threshold( grey, binimage, 200, 255, cv.CV_THRESH_BINARY_INV )
#        cv.ShowImage("binimage Image", binimage)
        #cv.WaitKey()
        """Find all contours."""
        stor = cv.CreateMemStorage()
        
        image04 = None
        cv.Zero(image04)    
        binimagematrix=vision.cv2array(binimage)
        cont, hierarchy = cv2.findContours(binimagematrix,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
        if (cont>1): 
            print 'length of cont %s' % len(cont)
            
        

        cnt = cont[0]
        moments = cv2.moments(cnt)
        (x,y),(MA,ma),angle = cv2.fitEllipse(cnt)
        box=cv.FitEllipse2(cv.fromarray(cnt))
        Ellipseorigen=box[0]
        Ellipselength1=box[1]
        Ellipsangle=box[2]
        print "---"
        print box
        print "---" 
        (xRect,yRect),(wRect,hRect),thetaRect = cv2.minAreaRect(cnt)

        if(self.xx==1):
            print x
            print y
            print angle*np.pi/180
            print " "
            print xRect
            print yRect
            print wRect
            print hRect       
            print thetaRect  
        
        length = 10
        x1 = int(xRect)
        y1 = int(yRect)
        x2 = int(xRect + np.sin(thetaRect*np.pi/180)*length)
        y2 = int(yRect + np.cos(thetaRect*np.pi/180)*length)
        
        binimage_rgb = cv.CreateMat(binimage.height, binimage.width, cv.CV_8UC3)
        cv.CvtColor(binimage,binimage_rgb,cv.CV_GRAY2RGB)
        cv.Line(binimage_rgb,(x1,y1),(x2,y2),(0,255,0),2,8)
#        cv.ShowImage('minAreaRect', binimage_rgb) 
        
        print   Ellipselength1[0]
        xf1 = int(Ellipseorigen[0])
        yf1 = int(Ellipseorigen[1])
        xf2 = int(xf1 + np.sin(Ellipsangle)*length)
        yf2 = int(yf1 + np.cos(Ellipsangle)*length)
        

        binimage_rgb2 = cv.CreateMat(binimage.height, binimage.width, cv.CV_8UC3)
        cv.CvtColor(binimage,binimage_rgb2,cv.CV_GRAY2RGB)

        cv.EllipseBox(binimage_rgb2,box,(0,255,0),2)
        cv.Line(binimage_rgb2,(xf1,yf1),(xf2,yf2),(0,255,0),2,8)
#        cv.ShowImage('fitEllipse', binimage_rgb2)


#ZZZZZ
#////////
#AAAA


        
        pubx = rospy.Publisher('vision', posmsg)
#        puby = rospy.Publisher('posmsg', posy)
        wert = float(232.0)
        wertt = float(999.0)
        if not rospy.is_shutdown():
        
            pubx.publish(Ellipseorigen[0], Ellipseorigen[1])
#            puby.publish(wertt)            
#            pubx.publish(Ellipseorigen[0])
#            puby.publish(Ellipseorigen[1])
           


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
    
    
    vision.init()
    
#    cv.NamedWindow("Image window", 1) #start a window
    rospy.init_node('vision_node')     #init the node
    rospy.Subscriber("/camera/rgb/image_color", Image, vision.callback) #subscribe to the camera topic, and start the function callback
    

    
    #cv.WaitKey(1000)    
    rospy.spin() #keps the node running

   

if __name__ == '__main__':
    
    vision=vision()
    main()

