#!/usr/bin/env python
import roslib
roslib.load_manifest('vision')

import cv,time,sys,rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


board_w=int(4)	# number of horizontal corners
board_h=int(3)	# number of vertical corners
n_boards=int(10) #nubers of pictures
z=0		# to print number of frames
succeses=0



#Get input data from user (out comepent if not needed)
"""
board_w=int(raw_input("Set the numbers of horizontal corners on the chessboard: "))
board_h=int(raw_input("Set the numbers of vertical corners on the chessboard: "))
n_boards=int(raw_input("Set numbers of images need for the calibration: "))
"""
board_n=board_w*board_h		# no of total corners
board_sz=(board_w,board_h)	#size of board

print "Now you are ready to take some pictures, remember to get as many deferent angels as possible"


#	creation of memory storages (Empty images/matrices)
image_points=cv.CreateMat(n_boards*board_n,2,cv.CV_32FC1)
object_points=cv.CreateMat(n_boards*board_n,3,cv.CV_32FC1)
point_counts=cv.CreateMat(n_boards,1,cv.CV_32SC1)
intrinsic_matrix=cv.CreateMat(3,3,cv.CV_32FC1)
distortion_coefficient=cv.CreateMat(5,1,cv.CV_32FC1)



def callback(image1):
    
    bridge = CvBridge() #make an object from the CvBridge class
    try:
        image = bridge.imgmsg_to_cv(image1, "bgr8") #convert from a ROS iamge to cv image
    except CvBridgeError, e:
        print e  # if it fails the, make error message   
 
    #	capture frames of specified properties and modification of matrix values
    i=0
    global succeses    
    global z
    if(succeses<n_boards):
        found=0
        #image=cv.QueryFrame(capture)        #capture image from video stream 
        gray_image=cv.CreateImage(cv.GetSize(image),8,1) #Create an empty 8 bit, 1 channel, gray image, with size of the captured image
        cv.CvtColor(image,gray_image,cv.CV_BGR2GRAY) #Change from RGB to gray image

        (found,corners)=cv.FindChessboardCorners(gray_image,board_sz,cv.CV_CALIB_CB_ADAPTIVE_THRESH| cv.CV_CALIB_CB_FILTER_QUADS) #Find the corners in the chessboard, and set found to true
        
        corners=cv.FindCornerSubPix(gray_image,corners,(11,11),(-1,-1),(cv.CV_TERMCRIT_EPS+cv.CV_TERMCRIT_ITER,30,0.1)) #Optimize the position of the corners in the chessboard
        if found==1:
            cv.DrawChessboardCorners(image,board_sz,corners,1) #Draw the found corners on the image
        key=-1
        key=cv.WaitKey(10)
	    # if got a good image,draw chessboard
        if found==1 and key!=-1:
            print "found frame number {0}".format(z+1)
		
            corner_count=len(corners) #Find the number of corners
            z=z+1
            # if got a good image, add to matrix
            if len(corners)==board_n:   #See if the number of corners match the number set
                step=succeses*board_n
                k=step
                for j in range(board_n):
                    #Save the position of the corners in a image_points
                    cv.Set2D(image_points,k,0,corners[j][0]) 
                    cv.Set2D(image_points,k,1,corners[j][1])

                    #Save the corners from alle images in one matrix
                    cv.Set2D(object_points,k,0,float(j)/float(board_w))
                    cv.Set2D(object_points,k,1,float(j)%float(board_w))
                    cv.Set2D(object_points,k,2,0.0)
                    k=k+1
                cv.Set2D(point_counts,succeses,0,board_n)
                succeses+=1
                
                time.sleep(2)
                print "-------------------------------------------------"
                print "\n"

        cv.ShowImage("Test Frame",image) #Send image to a window named "Test Frame"
        cv.WaitKey(33)

    if(n_boards==succeses):
        print "checking is fine	,all matrices are created"
        cv.DestroyWindow("Test Frame")

        # now assigning new matrices according to view_count
        object_points2=cv.CreateMat(succeses*board_n,3,cv.CV_32FC1)
        image_points2=cv.CreateMat(succeses*board_n,2,cv.CV_32FC1)
        point_counts2=cv.CreateMat(succeses,1,cv.CV_32SC1)

        #transfer points to matrices

        for i in range(succeses*board_n):
	        cv.Set2D(image_points2,i,0,cv.Get2D(image_points,i,0))
	        cv.Set2D(image_points2,i,1,cv.Get2D(image_points,i,1))
	        cv.Set2D(object_points2,i,0,cv.Get2D(object_points,i,0))
	        cv.Set2D(object_points2,i,1,cv.Get2D(object_points,i,1))
	        cv.Set2D(object_points2,i,2,cv.Get2D(object_points,i,2))
        for i in range(succeses):
	        cv.Set2D(point_counts2,i,0,cv.Get2D(point_counts,i,0))

        cv.Set2D(intrinsic_matrix,0,0,1.0)
        cv.Set2D(intrinsic_matrix,1,1,1.0)

        rcv = cv.CreateMat(n_boards, 3, cv.CV_64FC1)
        tcv = cv.CreateMat(n_boards, 3, cv.CV_64FC1)

        #In the above code, the data from the chessboards are reorganized to fit the cv.CalibrateCamera2 function

        print "checking camera calibration............."
        # camera calibration
        cv.CalibrateCamera2(object_points2,image_points2,point_counts2,cv.GetSize(image),intrinsic_matrix,distortion_coefficient,rcv,tcv,0)

        #object_points2 : Alle the corners from all the chessboards in a 3xN matrix
        #image_points2  : Alle the corners from all the images in a 2xN matrix
        #point_counts2  : Sum of alle corners in a 1xN matrix
        #cv.GetSize(image) : Returns the size of the image (e.g (600,800))
        #intrinsic_matrix : The camera matrix 3x3
        #distortion_coefficient : The distortion coefficients
        #rcv : Rotation matrix
        #tcv : Translation vector
        #0 : Flags for different setups, 0 is default 

        print " checking camera calibration.........................OK	"	

        # storing results in xml files
        cv.Save("camera_matrix.xml",intrinsic_matrix)
        cv.Save("distortion_vector.xml",distortion_coefficient)
        cv.Save("rot_matrix.xml",rcv)
        cv.Save("trans_vector.xml",tcv)



        print "------"
        print "Camera matrix:"
        print intrinsic_matrix[0,0],"   ",intrinsic_matrix[0,1],"   ",intrinsic_matrix[0,2]
        print intrinsic_matrix[1,0],"   ",intrinsic_matrix[1,1],"   ",intrinsic_matrix[1,2]
        print intrinsic_matrix[2,0],"   ",intrinsic_matrix[2,1],"   ",intrinsic_matrix[2,2]
        print "------"
        print "Distortion vector:"
        print distortion_coefficient[0,0],"   ",distortion_coefficient[1,0],"   ",distortion_coefficient[2,0],"   ",distortion_coefficient[3,0],"   ",distortion_coefficient[4,0]
        print "------"
        print "Rotation matrix:"
        print rcv[0,0],"   ",rcv[0,1],"   ",rcv[0,2]
        print rcv[1,0],"   ",rcv[1,1],"   ",rcv[1,2]
        print rcv[2,0],"   ",rcv[2,1],"   ",rcv[2,2]
        print "------"
        print "Translation vector:"
        print tcv[0,0],"   ",tcv[1,0],"   ",tcv[2,0]
        print "------"
        print " loaded all distortion parameters"

        succeses+=1
        time.sleep(8)

        print "now get ready, camera is switching on, hit ESC to exit"

    if(n_boards<succeses):

        # Loading from xml files
        intrinsic = cv.Load("camera_matrix.xml")
        distortion = cv.Load("distortion_vector.xml")

        #Create images to show the calihttp://www.cmu.edu/stores/computer/Hardware/LenovoProducts/lenovo-laptops.htmlbrated image vs the none calibrated
        mapx = cv.CreateImage( cv.GetSize(image), cv.IPL_DEPTH_32F, 1 );
        mapy = cv.CreateImage( cv.GetSize(image), cv.IPL_DEPTH_32F, 1 );
        cv.InitUndistortMap(intrinsic,distortion,mapx,mapy) #Create the two images
        cv.NamedWindow( "Undistort" )

        t = cv.GetImage(image)
        image2 = cv.GetImage(image)    
        cv.ShowImage( "Calibration", image)
        cv.Remap( t, image, mapx, mapy )
        cv.ShowImage("Undistort", image)
        cv.WaitKey (33)

    


def main():


   # global cv_image

    
    rospy.init_node('camera_calibration_keypush')     #init the node

    rospy.Subscriber("/camera/rgb/image_color", Image, callback) #subscribe to the camera topic, and start the function callback
    
    k = cv.WaitKey (33)
    if k == 27: #ESC key code
        print "everything is fine"
    else:
        rospy.spin() #keps the node running

   

if __name__ == '__main__':
    main()


