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
from numpy import linalg as LA

from vision.srv import start
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from robot_comm.srv import robot_SetJoints
from robot_comm.srv import robot_SetCartesian
from robot_comm.srv import robot_SetVacuum
from robot_comm.srv import robot_SetTool


class recover():

    def init(self):
            self.background = cv.LoadImage("/home/simplehands/Documents/hands/code/nodes/ABBnodes/vision/batt/background.jpg",cv.CV_LOAD_IMAGE_GRAYSCALE)
            #self.bat = cv.LoadImage("batt/2.jpg", cv.CV_LOAD_IMAGE_GRAYSCALE)
            self.diff = cv.CreateImage(cv.GetSize(self.background),self.background.depth, self.background.nChannels)
            self.size = cv.GetSize(self.background)

            #print self.workobject
            self.varx = 0
            

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


    def callback(self,image):       
        bridge = CvBridge() #make an object from the CvBridge class
        try:
            
            self.cv_image = bridge.imgmsg_to_cv(image, "bgr8") #convert from a ROS iamge to cv image
        except CvBridgeError, e:
            print e  # if it fails the, make error message

    def callbackHD(self,HDimage):       
        bridge = CvBridge() #make an object from the CvBridge class
        try:
            
            self.cv_imageHD = bridge.imgmsg_to_cv(HDimage, "bgr8") #convert from a ROS iamge to cv image
        except CvBridgeError, e:
            print e  # if it fails the, make error message       
        
        if self.varx <1:    
            background = cv.GetImage(self.cv_imageHD)
            cv.SaveImage("HDbackground.jpg", background)
            self.varx =self.varx+10

    def RotMat2Qua(self,v):
        q=[0.0,0.0,0.0,0.0]
        TOLERANCE = 0.0000000001
        trace = v[0] + v[4] + v[8] + 1.0
        if ((trace-1.0) >TOLERANCE):
            s = 0.5 / math.sqrt(trace)
            q[0] = 0.25 / s
            q[1] = ( v[7] - v[5] ) * s
            q[2] = ( v[2] - v[6] ) * s
            q[3] = ( v[3] - v[1] ) * s
        
        else:
            if ( v[0] > v[4] and v[0] > v[8] ):
                s = 2.0 * math.sqrt( 1.0 + v[0] - v[4] - v[8])
                q[0] = (v[7] - v[5] ) / s
                q[1] = 0.25 * s
                q[2] = (v[1] + v[3] ) / s
                q[3] = (v[2] + v[6] ) / s
            elif (v[4] > v[8]):
                s = 2.0 * math.sqrt( 1.0 + v[4] - v[0] - v[8])
                q[0] = (v[2] - v[6] ) / s
                q[1] = (v[1] + v[3] ) / s
                q[2] = 0.25 * s
                q[3] = (v[5] + v[7] ) / s
            else:
                s = 2.0 * math.sqrt( 1.0 + v[8] - v[0] - v[4] )
                q[0] = (v[1] - v[3] ) / s
                q[1] = (v[2] + v[6] ) / s
                q[2] = (v[5] + v[7] ) / s
                q[3] = 0.25 * s
        return q

    def multirotmatix(self,(a1, a2, a3, a4, a5, a6, a7, a8, a9, b1, b2, b3, b4, b5, b6, b7, b8, b9)):
        c=[0,0,0,0,0,0,0,0,0]
        c[0]= a1*b1 + a2*b4 + a3*b7
        c[1]= a1*b2 + a2*b5 + a3*b8
        c[2]= a1*b3 + a2*b6 + a3*b9
        c[3]= a4*b1 + a5*b4 + a6*b7
        c[4]= a4*b2 + a5*b5 + a6*b8
        c[5]= a4*b3 + a5*b6 + a6*b9
        c[6]= a7*b1 + a8*b4 + a9*b7
        c[7]= a7*b2 + a8*b5 + a9*b8
        c[8]= a7*b3 + a8*b6 + a9*b9
        #print c

        return(c)

    def Qua2RotMat(self,(a,b,c,d)):
    
        RT = [[a**2+b**2-c**2-d**2, 2*(b*c-a*d), 2*(b*d+a*c)],[2*(b*c+a*d), a**2-b**2+c**2-d**2,2*(c*d+a*b)],[2*(b*d-a*c),2*(c*d+a*b),a**2-b**2-c**2+d**2]]
        return(RT)

    def image2table(self,xx):
        u=xx[0]
        v=xx[1]
        z=xx[2]
        x=(7424788285795370035402623794365952*v - 257656381262487810712765849401327616*u - 73096170213812076980849026129517280*z + 226539073559552260591098173038464*u*z + 5062196849400538652919077370369*v*z + 4704544337763131935104844643459465216)/(835083430055756549538377664832*u + 21255644675418788265423927481756*v - 126625064647697976276659428356578912)
         
        y=-(2*(547901900092527097477553168895488*u - 33245339074439121088302078476041216*v - 6052470708179738678269494889736448*z - 515187885129436407756139745055*u*z + 28295397376263902196450886721412*v*z + 12846314413002244160428256079172739072))/(208770857513939137384594416208*u + 5313911168854697066355981870439*v - 31656266161924494069164857089144728)
        return (int(x),int(y),int(z))

    def table2robo(self,aa):
        #workobjectX: 808.5, workobjectY: -612.86, workobjectZ: 0.59,workobjectQ0: 0.7084, workobjectQX: 0.0003882, workobjectQY: -0.0003882,workobjectQZ: 0.7058
        b=[0,0,0,0,0,0,0]
        
        b[0]=-aa[1]+808.5
        b[1]=aa[0]-612.86
        b[2]=aa[2]+0.59
        b[3]=aa[3]#+0.7084
        b[4]=aa[4]#+0.0003882
        b[5]=aa[5]#-0.0003882
        b[6]=aa[6]#+0.7058

        return ((b[0],b[1],b[2],b[3],b[4],b[5],b[6]))


    def work2table(self,(x,y,z)):
        #workobjectX: 808.5, workobjectY: -612.86, workobjectZ: 0.59,workobjectQ0: 0.7084, workobjectQX: 0.0003882, workobjectQY: -0.0003882,workobjectQZ: 0.7058
        RV=recover.Qua2RotMat((self.workobject[3],self.workobject[4],self.workobject[5],self.workobject[6]))

        TM=np.array([[RV[0][0],RV[0][1],RV[0][2],self.workobject[0]],[RV[1][0],RV[1][1],RV[1][2],self.workobject[1]],[RV[2][0],RV[2][1],RV[2][2],self.workobject[2]],[0,0,0,1]])
        point=np.array([x,y,z,1]) 
         
        
        #print 'TM:', TM,'points:',np.transpose(np.array([x,y,z,1]))
        test=TM*np.transpose(np.array([x,y,z,1]))
        xtest=test[0][0]+test[0][1]+test[0][2]+test[0][3]
        ytest=test[1][0]+test[1][1]+test[1][2]+test[1][3]
        ztest=test[2][0]+test[2][1]+test[2][2]+test[2][3]
        #print 'xTEST:',xtest,'yTEST:',ytest,'zTEST:',ztest
        
        b=[0,0,0,0,0,0,0]
        
        b[0]=612.86+ytest
        b[1]=808.5-xtest
        b[2]=ztest
        

        return ((b[0],b[1],b[2]))

    def set_tool(self):
        rospy.wait_for_service('/robot_SetTool')
        try:
            robot_robot_SetTool = rospy.ServiceProxy('/robot_SetTool', robot_SetTool)
            resptool = robot_robot_SetTool(-81.40, 2.515, 159.57, 1.0, 0.0, 0.0, 0.0)     
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e    
        rospy.wait_for_service('/robot_SetTool')

        
    def set_robojoints(self,pos):
        rospy.wait_for_service('/robot_SetJoints')
        try:
            robotjoints = rospy.ServiceProxy('/robot_SetJoints', robot_SetJoints)
            resptool = robotjoints(pos[0],pos[1],pos[2],pos[3],pos[4],pos[5])     
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        rospy.wait_for_service('/robot_SetJoints')
        


    def set_robocoor(self,pos):
        rospy.wait_for_service('/robot_SetCartesian')
        try:
            robotcar = rospy.ServiceProxy('/robot_SetCartesian', robot_SetCartesian)
            resptool = robotcar(pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],pos[6])     
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        rospy.wait_for_service('/robot_SetCartesian')

    def set_suck(self,x):
        rospy.wait_for_service('/robot_SetVacuum')
        try:
            robotsuck = rospy.ServiceProxy('/robot_SetVacuum', robot_SetVacuum)
            resptool = robotsuck(x)     
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        rospy.wait_for_service('/robot_SetVacuum')
 

    def getimagecoor(self):
   
        self.bat = cv.GetImage(self.cv_image)
        self.gray = cv.CreateImage(cv.GetSize(self.bat),8, 1)

        cv.CvtColor(self.bat,self.gray,cv.CV_BGR2GRAY)
        cv.AbsDiff(self.background,self.gray,self.diff)
        
        #cv.ShowImage('self.diff',self.diff)
        cv.Threshold(self.diff,self.diff, 30,255,cv.CV_THRESH_BINARY)

        cv.Erode(self.diff,self.diff,None,1)
        cv.Dilate(self.diff,self.diff,None,1)

        '''ROI'''
        xroi= 100 # co-ordinate of top left vertex.
        yroi= 130 # co-ordinate of top left vertex.
        x2roi= 570 # width or region
        y2roi= 380 # height of region
     
        for x in range(self.size[0]):
            for y in range(self.size[1]):
                if x < xroi or x > x2roi or y < yroi or y > y2roi:
                    cv.Set2D(self.diff,y,x,[0,0,0])


        contours1, hierarchy = cv2.findContours(recover.cv2array(self.diff),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        points=np.zeros(shape=(len(contours1),2))
        

        areatemp=10
        #print len(contours1)
        for i in range(len(contours1)):
            cnt = contours1[i]
            M = cv2.moments(cnt)
            area = M['m00']
            
            if area > areatemp:
                
                center = [int(M['m10']/M['m00']),int(M['m01']/M['m00'])]
                areatemp=area




        #sumx=0
        #sumy=0
        #points=0
        #for x in range(self.size[0]):
        #    for y in range(self.size[1]):
        #        s=cv.Get2D(self.diff,y,x)
                
        #        if s[0] !=0:
        #            sumx=sumx+x
        #            sumy=sumy+y
        #            points=points+1
        batx=center[0]
        baty=center[1]
        #print 'x:',batx,' y:',baty
        #print 'points:',points,' y:',baty

        batz=recover.getZofpoint((batx,baty))
        #deff z

        #if  batx > 388 and batx < 460 and baty > 208 and baty < 273:
        #    batz=150
        #    batx=batx+5  
        #
        #elif batx > 366 and batx < 550 and baty > 146 and baty < 330:
        #    batz=126
        #else:
        #    batz=5.7

        print 'x:',batx,' y:',baty, 'z:',batz
        cv.NamedWindow('background', cv.CV_WINDOW_AUTOSIZE)
        cv.ShowImage('sub', self.diff)
         
        cv.WaitKey()
        return (batx,baty,batz)
    
    def getZofpoint(self,(x1,y1)):

        robocoor=recover.image2table((x1,y1,0))
        coortemp=recover.table2robo((robocoor[0],robocoor[1],0,0,0,0,0))
        xx=coortemp[0]
        yy=coortemp[1]
        #print 'robot x:', xx,'robot y:',yy
        RV=recover.Qua2RotMat((self.workobject[3],self.workobject[4],self.workobject[5],self.workobject[6]))
        #print 'rv:',RV
        TM=np.array([[RV[0][0],RV[0][1],RV[0][2],self.workobject[0]],[RV[1][0],RV[1][1],RV[1][2],self.workobject[1]],[RV[2][0],RV[2][1],RV[2][2],self.workobject[2]],[0,0,0,1]])
        point=np.array([xx,yy,0,1]) 
         
### TEST
        test=TM*np.transpose(np.array([0,0,0,1]))
        xtest=test[0][0]+test[0][1]+test[0][2]+test[0][3]
        ytest=test[1][0]+test[1][1]+test[1][2]+test[1][3]
        #print 'xTEST:',xtest,'yTEST:',ytest
###
  
        pointxy=LA.inv(TM)*np.transpose(point)
        #print 'LA.inv(TM):',LA.inv(TM)
        #print 'pointxy:', pointxy
        #print 'point:',np.transpose(point)
        x=pointxy[0][0]+pointxy[0][1]+pointxy[0][2]+pointxy[0][3]
        y=pointxy[1][0]+pointxy[1][1]+pointxy[1][2]+pointxy[1][3]
        print 'xWORKSPACE:',x,'yWORKSPACE:',y
        A=(-180,-105)#(-105,-180)
        B=(180,-105)#(-105,180)
        C=(183,260)#(260,183)
        D=(-183,260)#(260,-183)
        E=(-65,-65)#(-65,-65)   
        F=(65,-65)#(-65,65)
        G=(65,65)#(65,65)
        H=(-65,65)#(65,-65)

        #Linear interpolation
        ABx = A[0]+(y-A[0])*(B[0]-A[0])/(B[1]-A[1])
        ABy = A[1]+(x-A[0])*(B[1]-A[1])/(B[0]-A[0])
        BCx = B[0]+(y-B[0])*(C[0]-B[0])/(C[1]-B[1])
        BCy = B[1]+(x-B[0])*(C[1]-B[1])/(C[0]-B[0])
        CDx = C[0]+(y-C[0])*(D[0]-C[0])/(D[1]-C[1])
        CDy = C[1]+(x-C[0])*(D[1]-C[1])/(D[0]-C[0])
        DAx = D[0]+(y-D[0])*(A[0]-D[0])/(A[1]-D[1])
        DAy = D[1]+(x-D[0])*(A[1]-D[1])/(A[0]-D[0])
        
        EFx = E[0]+(y-E[0])*(F[0]-E[0])/(F[1]-E[1])
        EFy = E[1]+(x-E[0])*(F[1]-E[1])/(F[0]-E[0])
        FGx = F[0]+(y-F[0])*(G[0]-F[0])/(G[1]-F[1])
        FGy = F[1]+(x-F[0])*(G[1]-F[1])/(G[0]-F[0])
        GHx = G[0]+(y-G[0])*(H[0]-G[0])/(H[1]-G[1])
        GHy = G[1]+(x-G[0])*(H[1]-G[1])/(H[0]-G[0])
        HEx = H[0]+(y-H[0])*(E[0]-H[0])/(E[1]-H[1])
        HEy = H[1]+(x-H[0])*(E[1]-H[1])/(E[0]-H[0])

        #      B  __________________ C
        #        |                  |
        #        |  PLATFORM        |
        #        |                  |
        #        | F______G         |
        #        | | x    |         |
        #        | | |    |         |
        #        | | |__y |         |
        #        | |______|         |
        #        | E      H         |
        #        |__________________|
        #      A                     D

        print 'EFy:',EFy,'GHy:',GHy,'FGx:',FGx,'HEx:',HEx,'ABy:',ABy,'CDy:',CDy,'BCx:',BCx,'DAx:',DAx
        if   y > EFy and y < GHy and x < FGx and x > HEx:
            batz=150
            #batx=batx+5  
        
        elif y > ABy and y < CDy and x < BCx and x > DAx:
            batz=126
        else:
            batz=5.7
        #x0+(y-y0)*(x1-x0)/(y1-y0)

        return(batz)

    def getor(self):
        time.sleep(2)
        self.batHD = cv.GetImage(self.cv_imageHD)    
        #cv.ShowImage('HD', self.batHD)
        #cv.SaveImage("HDbat.jpg", self.batHD)
        cv.WaitKey(1)

        size=cv.GetSize(self.batHD)
        temp2 = cv.CreateImage(cv.GetSize(self.batHD),cv.IPL_DEPTH_8U, 3)
        grey = cv.CreateImage(cv.GetSize(self.batHD),8, 1)

    #    '''ROI'''
    #    xroi= 100 # co-ordinate of top left vertex.
    #    yroi= 90 # co-ordinate of top left vertex.
    #    x2roi= 570 # width or region
    #    y2roi= 330 # height of region
    # 
    #    for x in range(size[0]):
    #        for y in range(size[1]):
    #            if x < xroi or x > x2roi or y < yroi or y > y2roi:
    #                cv.Set2D(image,y,x,[0,0,0])


        '''Thresholding'''
        t= [255,255,255]
        x=0
        y=0
        
        while x <size[0]:
            while y <size[1]:
                s=cv.Get2D(self.batHD,y,x)
                #print "x:",x," y:",y," s:",s
                if (s[0]> 65 ) : #B
                    if (s[1] > 56): #G
                        if (s[2] > 39): #R
                            cv.Set2D(temp2,y,x,t)
                y+=1
            x+=1
            y=0

        cv.ShowImage('window', temp2) #Show the image
        cv.WaitKey(1)
            
        cv.CvtColor(temp2,grey,cv.CV_BGR2GRAY)
        cv.Zero(temp2)
        #cv.Erode(grey,grey,None,1)
        cv.Dilate(grey,grey,None,1)
        #cv.ShowImage('window22', grey) #Show the image
        cv.WaitKey(1)    
        
        contours, hierarchy = cv2.findContours(recover.cv2array(grey),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        points=np.zeros(shape=(len(contours),2))
        cv.Zero(grey)

        areatemp=10
        #print len(contours)
        for i in range(len(contours)):
            cnt = contours[i]
            M = cv2.moments(cnt)
            area = M['m00']
            
            if area > areatemp:
                
                center = [int(M['m10']/M['m00']),int(M['m01']/M['m00'])]
                areatemp=area
                batcont=contours[i]


        rect = cv2.minAreaRect(batcont)
        box  = cv2.cv.BoxPoints(rect)
       
        box2=[(0,0),(0,0),(0,0),(0,0)]
     
        box2[0]=((box[0][0]+box[1][0])/2,(box[0][1]+box[1][1])/2)
        box2[1]=((box[1][0]+box[2][0])/2,(box[1][1]+box[2][1])/2)
        box2[2]=((box[2][0]+box[3][0])/2,(box[2][1]+box[3][1])/2)
        box2[3]=((box[3][0]+box[0][0])/2,(box[3][1]+box[0][1])/2)
        lengthtest=0
     
        length = math.sqrt((box2[0][0]-box2[2][0])**2 + (box2[0][1]-box2[2][1])**2)
        if length > lengthtest:
            point1=box2[0]
            point2=box2[2]
            lengthtest=length
        length = math.sqrt((box2[1][0]-box2[3][0])**2 + (box2[1][1]-box2[3][1])**2)
        if length > lengthtest:
            point1=box2[1]
            point2=box2[3]
            lengthtest=length
        length = math.sqrt((box2[2][0]-box2[0][0])**2 + (box2[2][1]-box2[0][1])**2)
        if length > lengthtest:
            point1=box2[2]
            point2=box2[0]
            lengthtest=length
        length = math.sqrt((box2[3][0]-box2[1][0])**2 + (box2[3][1]-box2[1][1])**2)
        if length > lengthtest:
            point1=box2[3]
            point2=box2[1]
            lengthtest=length

        cv.Circle(self.batHD,(int(point1[0]),int(point1[1])),5,(255,255,255),-1,8,0)
        cv.Circle(self.batHD,(int(point2[0]),int(point2[1])),5,(255,0,0),-1,8,0)
        cv.Line(self.batHD,(int(point1[0]),int(point1[1])),(int(point2[0]),int(point2[1])),(255,0,),2,8)
        cv.Circle(self.batHD,(int(center[0]),int(center[1])),5,(255,0,0),-1,8,0)    
        
        if point1[1]>point2[1]:
            angle=math.pi/2-np.arcsin((point2[0]-point1[0]) / lengthtest)
        else:
            angle=(np.arcsin((point2[0]-point1[0]) / lengthtest))-math.pi/2


        #print 'x:',320-center[0],'y:',244-center[1],'angle:',angle
        #print 'x in mm:',(320-center[0])*0.384732,'y in mm:',(244-center[1])*0.384732
        #cv.Line(self.batHD, (int(x),int(y)), (int(x+np.cos(angle)*200),int(y+np.sin(angle)*200)), (255,0,0), 2, 8, 0)
####320 244##
        cv.Circle(self.batHD,(int(center[0]),int(center[1])),2,(0,0,0),-1,8,0)
        cv.Circle(self.batHD,(int(320),int(244)),2,(0,255,0),-1,8,0)

        cv.ShowImage('image', self.batHD) #Show the image
        #cv.SaveImage("self.batHD.jpg", self.batHD)
        cv.WaitKey(0)
        
        return ((320-center[0])*0.384732,(244-center[1])*0.384732,angle)




    
    def run(self,image):
        self.workobject = rospy.get_param('ABB/workobject')
        recover.set_tool()
        rospy.Subscriber("/camera/rgb/image_color", Image, recover.callback)
        rospy.Subscriber("/vis_hand/camera/image_raw", Image, recover.callbackHD)
        recover.set_robojoints((0.0, 0.0, 0.0, 0.0, 90.0, 0.0))
        recover.set_robojoints((90.0, 0.0, 0.0, 0.0, 90.0, 0.0))
        workobjectintable=recover.work2table((0,0,0))

        coor=recover.getimagecoor()
        #print 'coor:',coor
        robocoor=recover.image2table((coor))
        #print 'robocoor: ',robocoor
        #print 'workobject:',workobjectintable

        recover.set_robocoor((workobjectintable[0],workobjectintable[1], 300.0, 0.0, -0.706, -0.708, 0.0))
        recover.set_robocoor((robocoor[0],robocoor[1],300.0,0.0, -0.706, -0.708, 0.0))
        recover.set_robocoor((robocoor[0],robocoor[1],robocoor[2],0.0, -0.706, -0.708, 0.0))



        #recover.set_robocoor((360.5,288.6,5.7,0.0, -0.706, -0.708, 0.0)) 
             
        recover.set_suck(1)
        time.sleep(1)
        recover.set_robocoor((robocoor[0],robocoor[1],300.0,0.0, -0.706, -0.708, 0.0))
        recover.set_robocoor((workobjectintable[0],workobjectintable[1], 300.0, 0.0, -0.706, -0.708, 0.0))
        recover.set_robocoor((58.2,438.8,660.3,0.707, 0.707, 0.002, 0.001))
        orientation=recover.getor()
        #print 'orientation:',orientation 
        rotmatrix=recover.multirotmatix((0, 1, 0, 1, 0, 0, 0, 0, -1,np.cos(orientation[2]),np.sin(orientation[2]),0,-np.sin(orientation[2]),np.cos(orientation[2]),0,0,0,1))#180omX 90omZ and angleOmZ
        
        quat=recover.RotMat2Qua((rotmatrix[0],rotmatrix[1],rotmatrix[2],rotmatrix[3],rotmatrix[4], rotmatrix[5],rotmatrix[6],rotmatrix[7],rotmatrix[8]))       
#        print 'quat:',quat
        #recover.set_robocoor((360.5+orientation[1],303.6+orientation[0],5.7, float(quat[0]),float(quat[1]),float(quat[2]),float(quat[3])))
        
        
        #time.sleep(0.5)
      
        #recover.set_suck(0)
        #recover.set_robocoor((360.5-orientation[1],288.6+orientation[0],50.7, float(quat[0]),float(quat[1]),float(quat[2]),float(quat[3])))
        #recover.set_robocoor((360.5-orientation[1],288.6+orientation[0],50.7,0.0, -0.706, -0.708, 0.0))
        #recover.set_robocoor((360.5-orientation[1],288.6+orientation[0],5.7,0.0, -0.706, -0.708, 0.0))
        #recover.set_suck(1)
        #time.sleep(1)
        recover.set_robocoor((workobjectintable[0],workobjectintable[1], 300.0, 0.0, -0.706, -0.708, 0.0))

        
        bin=recover.work2table((-14.3,145.9,15))
        #print 'bin:',bin
        recover.set_robocoor((bin[0]-orientation[1],bin[1]+orientation[0],bin[2], float(quat[0]),float(quat[1]),float(quat[2]),float(quat[3])))
        time.sleep(0.5)
        recover.set_suck(0)
        cv.DestroyAllWindows()
        return ((1,0,0,0,0,0,0))
        
def main():
    recover.init()
    s = rospy.Service('abbrecover', start, recover.run)
    rospy.spin() #keps the node running

if __name__ == '__main__':
    rospy.init_node('recover')
    recover=recover()

    main()







