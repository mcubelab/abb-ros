#!/usr/bin/env python
import roslib
roslib.load_manifest('vision')
roslib.load_manifest('robot_comm')

import cv,time,sys,rospy, random
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from robot_comm.srv import robot_GetCartesian
from robot_comm.srv import robot_SetCartesian
from robot_comm.srv import robot_SetTool
class calib():

    def init(self):
        self.succeses=0
        self.board_w=4	# number of horizontal corners
        self.board_h=3	# number of vertical corners 
        self.board_sz=(self.board_w,self.board_h)	#size of board
        self.corners=np.zeros(shape=(12,2))        
        self.robopoints=np.zeros(shape=(12,3))
        self.temp2=np.zeros(shape=(1,3))
        self.allrobocoor = [0,0,0]
        self.allcamcoor = [0,0]
        self.count1 =0
        self.count2 =0
       
        self.platepos =   [[-90, -45, 28.575, 1],[-90, -15, 28.575 ,1],[-90, 15, 28.575 ,1],[-90, 45, 28.575 ,1],[-120, -45, 28.575 ,1],[-120, -15, 28.575 ,1],[-120, 15, 28.575 ,1],[-120, 45, 28.575 ,1],[-150, -45, 28.575 ,1],[-150, -15, 28.575 ,1],[-150, 15, 28.575 ,1],[-150, 45, 28.575 ,1]]
        
        #self.gotopoints = [[610.3,490.2,645.5],[731.3,490.6,645.3],[545.4,561.4,645.6],[319.8,510.3,413.4],[538.9,479.4,413.1],[917.2,599.5,412.6],[570.8,392.2,412.8],[570.8,392.2,276.9],[239.6,686.2,277.2],[240.2,497.1,277.2],[564,364.6,276.6],[585.2,427.9,164.9],[184,618.1,165.3],[321.4,618.5,165.1],[496.6,411.9,235.9]] #18
        self.gotopoints = [[610.3,490.2,645.5],[731.3,490.6,645.3],[545.4,561.4,645.6],[319.8,510.3,413.4],[538.9,479.4,413.1],[917.2,599.5,412.6],[570.8,392.2,412.8],[570.8,392.2,276.9],[1025.9,659.1,276.4],[610.3,490.2,645.5],[240.2,497.1,277.2],[564,364.6,276.6],[585.2,427.9,164.9],[184,618.1,165.3],[321.4,618.5,165.1],[496.6,411.9,235.9],[1043.4,649.3,235.3],[1043.1,649.2,113.1],[738.5,447.9,777.5],[581.6,407.3,793.8]] #21
        #self.gotopoints = [[610.3,490.2,645.5],[731.3,490.6,645.3],[545.4,561.4,645.6],[432.4,608.2,645.4],[319.8,510.3,413.4],[538.9,479.4,413.1],[917.2,599.5,412.6],[570.8,392.2,412.8],[570.8,392.2,276.9],[1025.9,659.1,276.4],[239.6,686.2,277.2],[240.2,497.1,277.2],[564,364.6,276.6],[585.2,427.9,164.9],[184,618.1,165.3],[321.4,618.5,165.1],[496.6,411.9,235.9],[1043.4,649.3,235.3],[1043.1,649.2,113.1],[497.7,447.9,777.8],[738.5,522.8,777.5],[581.6,407.3,793.8]] #22
    #---end of init---


    def callback(self,image1):
        
        bridge = CvBridge() #make an object from the CvBridge class
        try:
            image = bridge.imgmsg_to_cv(image1, "bgr8") #convert from a ROS iamge to cv image
        except CvBridgeError, e:
            print e  # if it fails the, make error message   
     
        #	capture frames of specified properties and modification of matrix values
        i=0
        n_boards= self.board_w*self.board_h
        if(self.succeses<n_boards):
            found=0
            #image=cv.QueryFrame(capture)        #capture image from video stream 
            gray_image=cv.CreateImage(cv.GetSize(image),8,1) #Create an empty 8 bit, 1 channel, gray image, with size of the captured image
            cv.CvtColor(image,gray_image,cv.CV_BGR2GRAY) #Change from RGB to gray image

            (found,self.corners)=cv.FindChessboardCorners(gray_image,self.board_sz,cv.CV_CALIB_CB_ADAPTIVE_THRESH| cv.CV_CALIB_CB_FILTER_QUADS) #Find the corners in the chessboard, and set found to true
            
            self.corners=cv.FindCornerSubPix(gray_image,self.corners,(11,11),(-1,-1),(cv.CV_TERMCRIT_EPS+cv.CV_TERMCRIT_ITER,30,0.1)) #Optimize the position of the corners in the chessboard

    #---end of callback---          

    def moveRobo(self,i):
        q0= 0.0
        qx= -0.706
        qy= -0.708
        qz= 0.0
        x = self.gotopoints[i][0]
        y = self.gotopoints[i][1]
        z = self.gotopoints[i][2]

        coor=(x,y,z, q0, qx, qy, qz)
                
        calib.move_to_coor(coor)
    #---end of moveRobo---


    def GetImageCoor(self):
        rospy.Subscriber("/camera/rgb/image_color", Image, calib.callback) #subscribe to the camera topic, and start the function callback
    #---end of GetImageCoor---

   
    def GetRoboCoor(self):
        rospy.wait_for_service('robot_GetCartesian')
        try:
            robot_getCar = rospy.ServiceProxy('/robot_GetCartesian', robot_GetCartesian)
            data = robot_getCar()     
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
       
        a=-data.q0
        b=-data.qx
        c=-data.qy
        d=-data.qz
        
        self.RT = [[a**2+b**2-c**2-d**2, 2*(b*c-a*d), 2*(b*d+a*c),data.x],[2*(b*c+a*d), a**2-b**2+c**2-d**2,2*(c*d+a*b),data.y],[2*(b*d-a*c),2*(c*d+a*b),a**2-b**2-c**2+d**2,data.z]]
        
        #print self.RT
        #print np.transpose(self.platepos[0])
        #print self.platepos[0]
        for exxt in range(12):
            temp= self.RT*np.transpose(self.platepos[exxt])
            #print temp
            self.robopoints[exxt][0]=temp[0][0]+temp[0][1]+temp[0][2] + temp[0][3]
            self.robopoints[exxt][1]=temp[1][0]+temp[1][1]+temp[1][2] + temp[1][3]
            self.robopoints[exxt][2]=temp[2][0]+temp[2][1]+temp[2][2] + temp[2][3]
        #print self.robopoints
    #---GetRoboCoor---
    
    
    def savearrays(self):
        print self.corners
        if self.count1 <= 0:
            self.allrobocoor = self.robopoints
            self.allcamcoor = self.corners
            self.count1 = 1   
        else:
            #print self.allrobocoor
            self.allrobocoor = np.append(self.allrobocoor,self.robopoints,0)
            self.allcamcoor = np.append(self.allcamcoor,self.corners,0)
            
        print "------"
        #print self.allrobocoor
        #print type(self.allrobocoor)
        #print self.allcamcoor
        #print type(self.allcamcoor)
    
        np.savetxt('robo.txt', self.allrobocoor)
        np.savetxt('camera.txt', self.allcamcoor)
    #--end of savearrays

    def move_to_coor(self,coor):
        print coor
        rospy.wait_for_service('robot_SetCartesian')
        try:
            robot_SetCartesian1 = rospy.ServiceProxy('/robot_SetCartesian', robot_SetCartesian)
            resp1 = robot_SetCartesian1(coor[0], coor[1],coor[2],coor[3],coor[4],coor[5],coor[6])     
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    #--end of move_to_coor
   
 
    def set_tool(self):
        rospy.wait_for_service('robot_SetTool')
        try:
            robot_robot_SetTool = rospy.ServiceProxy('/robot_SetTool', robot_SetTool)
            resptool = robot_robot_SetTool(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)     
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e    
        rospy.wait_for_service('robot_SetTool')

    def main(self):
        rospy.init_node('caliExter')     #init the node   
        calib.init()
        calib.set_tool()
        start=(612.2, 360.7, 640.7, 0.0, -0.706, -0.708, 0.0)
        
        calib.move_to_coor(start)
        for i in range(21): #15
            print i  
            calib.savearrays()
            time.sleep(1)
            calib.moveRobo(i)
            #raw_input("Press Enter when robot is ready..")
            calib.GetImageCoor()
            time.sleep(4)
            calib.GetRoboCoor()
            time.sleep(1)

            self.count2=0
        print "you are done!, Ctrl - c to exit"
        rospy.spin() #keps the node running
    #---end of main---
   

if __name__ == '__main__':
    calib=calib()
    calib.main()


#612.2, 360.7, 540.7, 0.0, 0.7071, 0.7071, 0.0















































