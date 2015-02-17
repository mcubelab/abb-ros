#!/usr/bin/env python
import roslib; roslib.load_manifest('vision')
import roslib; roslib.load_manifest('robot_comm')
import sys
import rospy
import cv
import cv2
import math
import time
import numpy as np

from vision.srv import start
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from robot_comm.srv import robot_SetJoints

class getcoor():

    def init(self):
        self.xya = [0,0,0]      
        self.temp=0
        
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

    def callback(self,image1):     
        bridge = CvBridge() #make an object from the CvBridge class
        try:
            
            image = bridge.imgmsg_to_cv(image1, "bgr8") #convert from a ROS iamge to cv image
        except CvBridgeError, e:
            print e  # if it fails the, make error message
           
        #pic = cv.SaveImage("testtest.jpg", image)
        #print "done"
        

        imgHsvThr = cv.CreateImage(cv.GetSize(image),cv.IPL_DEPTH_8U, 3)
        grey = cv.CreateImage(cv.GetSize(image),8, 1)

        size=cv.GetSize(image)
        xsize=int(size[0])
        ysize=int(size[1])

        '''ROI'''
        xroi= 100 # co-ordinate of top left vertex.
        yroi= 90 # co-ordinate of top left vertex.
        x2roi= 570 # width or region
        y2roi= 330 # height of region
     
        for x in range(size[0]):
            for y in range(size[1]):
                if x < xroi or x > x2roi or y < yroi or y > y2roi:
                    cv.Set2D(image,y,x,[0,0,0])



        '''Thresholding'''
        t= [255,255,255]
        x=0
        y=0
        
        while x <xsize:
            while y <ysize:
                s=cv.Get2D(image,y,x)
                #print "x:",x," y:",y," s:",s
                if (s[0]< 113 ) : #B
                    if (s[1] < 59): #G
                        if (s[2] > 70): #R
                            cv.Set2D(imgHsvThr,y,x,t)
                y+=1
            x+=1
            y=0

        cv.CvtColor(imgHsvThr,grey,cv.CV_BGR2GRAY)
        cv.Erode(grey,grey,None,2)
        cv.Dilate(grey,grey,None,2)
        #cv.SaveImage("grey.jpg", grey)
        cv.ShowImage('window', grey) #Show the image
        contours, hierarchy = cv2.findContours(getcoor.cv2array(grey),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        points=np.zeros(shape=(len(contours),2))
        theline=np.zeros(shape=(3,2))
        #print 'cont', len(contours)
        for i in range(len(contours)):
            cnt = contours[i]
            M = cv2.moments(cnt)
            points[i] = [int(M['m10']/M['m00']),int(M['m01']/M['m00'])]
            #print points[i]

        lengthtest = 0
        #find the relasion between the points
        length = math.sqrt((points[0][0]-points[1][0])**2 + (points[0][1]-points[1][1])**2)
        if length > lengthtest:
            lengthtest = length
            theline[0]=points[0]
            theline[1]=points[1]
            theline[2]=points[2]
        length = math.sqrt((points[1][0]-points[2][0])**2 + (points[1][1]-points[2][1])**2)
        if length > lengthtest:
            lengthtest = length
            theline[0]=points[1]
            theline[1]=points[2]
            theline[2]=points[0]
        length = math.sqrt((points[2][0]-points[0][0])**2 + (points[2][1]-points[0][1])**2)
        if length > lengthtest:
            lengthtest = length
            theline[0]=points[0]
            theline[1]=points[2]
            theline[2]=points[1]

        length = math.sqrt((theline[0][0]-theline[2][0])**2 + (theline[0][1]-theline[2][1])**2)

        if length < lengthtest:
            lengthtest = length
            startpoint_x=theline[0][0]
            startpoint_y=theline[0][1]
            endpoitn_x=theline[1][0]
            endpoitn_y=theline[1][1]
        length1 = math.sqrt((theline[1][0]-theline[2][0])**2 + (theline[1][0]-theline[2][1])**2)

        if length1 < lengthtest:
            lengthtest = length
            startpoint_x=theline[0][0]
            startpoint_y=theline[0][1]
            endpoitn_x=theline[1][0]
            endpoitn_y=theline[1][1]

        #print startpoint_x
        #print startpoint_y
        #print endpoitn_x
        #print endpoitn_y


        startend=math.sqrt((startpoint_x-endpoitn_x)**2 + (startpoint_y-endpoitn_y)**2)

        angle = np.arcsin((startpoint_x-endpoitn_x)/startend)

        #print "Angle:"
        #print angle
        #print "endpoitn x,y"
        #print endpoitn_x,endpoitn_y
        #print "startpoint"
        #print startpoint_x,startpoint_y
        #print "length"
        #print startend
        vinkel=1.05

        if angle <0:
            angle = math.pi+angle
            workspace = [endpoitn_x - np.cos(angle-vinkel)*125.7236, endpoitn_y + np.sin(angle-vinkel)*125.7236]
            print endpoitn_x
            print endpoitn_y
        else:
            workspace = [startpoint_x - np.cos(angle-vinkel)*125.7236, startpoint_y + np.sin(angle-vinkel)*125.7236]
            print startpoint_x
            print startpoint_y



        print 'workspace',workspace
        line10 = [workspace[0]+ 50*np.sin(angle),workspace[1]+50*np.cos(angle)]
        line20 = [workspace[0]+ 50*np.sin(angle-math.pi/2),workspace[1]+50*np.cos(angle-math.pi/2)]

        cv.Line(image,(int(workspace[0]),int(workspace[1])),(int(line20[0]),int(line20[1])),(255,0,),2,8)
        cv.Line(image,(int(workspace[0]),int(workspace[1])),(int(line10[0]),int(line10[1])),(0,0,255),2,8)

        cv.Line(image,(int(startpoint_x),int(startpoint_y)),(int(endpoitn_x),int(endpoitn_y)),(0,255,0),2,8)
        cv.SaveImage("line.jpg", image)
        cv.ShowImage('line', image) #Show the image
        
        self.xya = [workspace[0], workspace[1], angle]
        
        cv.ShowImage('window', imgHsvThr) #Show the image
        cv.WaitKey(0)
        

    def image2robot(self,xx):
        u=xx[0]
        v=xx[1]
        z=xx[2]
        
        x=-(1108247298312935287526774848268140544*u - 13558403624322441253996072380721152*v + 309671842211463072322889051727411968*z - 969766111049972937798597887363136*u*z + 5631218880686565157704721049251*v*z - 30010047962102955462018505029206933504)/(185967444642483348237178060816*u + 11401860756910287882601310052902*v - 518546573771699092136342527698204336)
         
        y=(3080572306024148572938569584766976*u + 554275593630345315260108816977416192*v + 135116306945816030547670888478785024*z - 2677595013864698690244591008610*u*z - 483876206221130888523454207321728*v*z - 211030534154996511862259014094556921856)/(92983722321241674118589030408*u + 5700930378455143941300655026451*v - 259273286885849546068171263849102168)
        
        '''
        x=(7424788285795370035402623794365952*v - 257656381262487810712765849401327616*u - 73096170213812076980849026129517280*z + 226539073559552260591098173038464*u*z + 5062196849400538652919077370369*v*z + 4704544337763131935104844643459465216)/(835083430055756549538377664832*u + 21255644675418788265423927481756*v - 126625064647697976276659428356578912)
         
        y=(3080572306024148572938569584766976*u + 554275593630345315260108816977416192*v + 135116306945816030547670888478785024*z - 2677595013864698690244591008610*u*z - 483876206221130888523454207321728*v*z - 211030534154996511862259014094556921856)/(92983722321241674118589030408*u + 5700930378455143941300655026451*v - 259273286885849546068171263849102168)
        '''
        return (x,y,z)
        
        
    def set_robojoints(self,pos):
        rospy.wait_for_service('/robot_SetJoints')
        try:
            robotjoints = rospy.ServiceProxy('/robot_SetJoints', robot_SetJoints)
            resptool = robotjoints(pos[0],pos[1],pos[2],pos[3],pos[4],pos[5])     
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        rospy.wait_for_service('/robot_SetJoints')
        
  
        
    def table2robo(self,aa):
        #workobjectX: 808.5, workobjectY: -612.86, workobjectZ: 0.59,workobjectQ0: 0.7084, workobjectQX: 0.0003882, workobjectQY: -0.0003882,workobjectQZ: 0.7058
        b=[0,0,0,0,0,0,0]
        
        b[0]=-aa[1]+811.6253
        b[1]=aa[0]-611.3932
        b[2]=aa[2]+0.59
        b[3]=aa[3]#+0.7084
        b[4]=aa[4]#+0.0003882
        b[5]=aa[5]#-0.0003882
        b[6]=aa[6]#+0.7058

        return ((b[0],b[1],b[2],b[3],b[4],b[5],b[6]))

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


    def run(self,image):
        getcoor.set_robojoints((90.0, 0.0, 0.0, 0.0, 90.0, 0.0))
        out = rospy.Subscriber("/camera/rgb/image_color", Image, getcoor.callback)
        
        rospy.sleep(5.0)
        #print self.xya
        #print "this is it: %s"%(req.a)
        quat =getcoor.RotMat2Qua((np.cos(math.pi/2-self.xya[2])*np.cos(math.pi), -np.cos(math.pi)*np.sin(math.pi/2-self.xya[2]), np.sin(math.pi), np.sin(math.pi/2-self.xya[2]), np.cos(math.pi/2-self.xya[2]), 0, -np.cos(math.pi/2-self.xya[2])*np.sin(math.pi), np.sin(math.pi/2-self.xya[2])*np.sin(math.pi), np.cos(math.pi)))
        
        

        temp= getcoor.image2robot((float(self.xya[0]),float(self.xya[1]),float(155)))

        


        print getcoor.table2robo([temp[0], temp[1], temp[2], float(quat[0]),float(quat[1]), float(quat[2]),float(quat[3])])
        #rospy.set_param('ABB', {'workobject': getcoor.table2robo([temp[0], temp[1], temp[2], float(quat[0]),float(quat[1]), float(quat[2]),float(quat[3])])})
        self.temp=1
        #print "yes!"
        return (getcoor.table2robo((temp[0], temp[1], temp[2], quat[0], quat[1], quat[2], quat[3])))

           

    def main(self):
        
        
        s = rospy.Service('abbgetWorkobject', start, getcoor.run)
        #getcoor.run('test')
        #print self.temp
        rospy.spin() #keps the node running

if __name__ == '__main__':
    rospy.init_node('getworkspace')
    getcoor=getcoor()
    getcoor.init()
    cv.NamedWindow('a_window', cv.CV_WINDOW_AUTOSIZE)
    getcoor.main()




