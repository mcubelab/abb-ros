#!/usr/bin/env python
import roslib; roslib.load_manifest('vision')

import sys
import rospy
import cv
import cv2
import math
import numpy
import time

from vision.srv import connectors

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class fc():

    def init(self):
        inputs=numpy.loadtxt('images.txt')
        targets =numpy.loadtxt('targets.txt')
        rospy.Subscriber("/vis_hand/camera/image_raw", Image, fc.callbackHD)

        # The number of elements in an input vector, i.e. the number of nodes
        # in the input layer of the network.
        ninputs = len(inputs[0])
        # 8 hidden nodes.
        nhidden = 8
        
        # Nubers of output nodes, in this project, if the connectors are up, down or not there.
        noutput = 3
        
        # Create an array of desired layer sizes for the neural network
        layers = numpy.array([ninputs, nhidden, noutput])

        # Create the neural network
        self.nnet = cv2.ANN_MLP(layers)

        # Some parameters for learning.  Step size is the gradient step size
        # for backpropogation.
        step_size = 0.01

        # Momentum can be ignored.
        momentum = 0.0

        # Max steps of training
        nsteps = 10000

        # Error threshold for halting training
        max_err = 0.0001

        # When to stop: whichever comes first, count or error
        condition = cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS

        # Tuple of termination criteria: first condition, then # steps, then
        # error tolerance second and third things are ignored if not implied
        # by condition
        criteria = (condition, nsteps, max_err)

        # params is a dictionary with relevant things for NNet training.
        params = dict( term_crit = criteria, 
                       train_method = cv2.ANN_MLP_TRAIN_PARAMS_BACKPROP, 
                       bp_dw_scale = step_size, 
                       bp_moment_scale = momentum )

        # Train the network
        num_iter = self.nnet.train(inputs, targets,
                              None, params=params)

        # Create a matrix of predictions
        predictions = numpy.empty_like(targets)

        # See how the network did.
        self.nnet.predict(inputs, predictions)

        # Compute sum of squared errors
        sse = numpy.sum( (targets - predictions)**2 )

        # Compute # correct
        true_labels = numpy.argmax( targets, axis=0 )
        pred_labels = numpy.argmax( predictions, axis=0 )
        num_correct = numpy.sum( true_labels == pred_labels )

        print 'ran for %d iterations' % num_iter
        print 'inputs:'
        print inputs
        print 'targets:'
        print targets
        print 'predictions:'
        print predictions
        print 'sum sq. err:', sse
        print 'accuracy:', float(num_correct) / len(true_labels)

    def callbackHD(self,HDimage):       
        bridge = CvBridge() #make an object from the CvBridge class
        try:
            
            self.cv_imageHD = bridge.imgmsg_to_cv(HDimage, "bgr8") #convert from a ROS iamge to cv image
        except CvBridgeError, e:
            print e  # if it fails the, make error message  

    def run(self):
        ROI=(308,131,34,173)
        image = cv.GetImage(self.cv_imageHD)

        size = cv.GetSize(image)
        im_thres=cv.CreateImage(size,8,1)
        cv.CvtColor(im, im, cv.CV_BGR2HSV)
        t=[255,255,255]
        x=0
        y=0
        while x <size[0]:
            while y <size[1]:
                s=cv.Get2D(im,y,x)
                if (s[0]>= 36/2 and s[0]<=100/2 ) : #H
                    if (s[1] >= 0 and s[1]<=185): #S
                        if (s[2] >= 100 and s[2]<=255): #V
                            cv.Set2D(im_thres,y,x,t)
                y+=1
            x+=1
            y=0
        x=0
        y=0
        cv.SetImageROI(im_thres,ROI)
        cv.SetImageROI(im,ROI)
    
        tt=0
        sizethres= cv.GetSize(im_thres)
        for i in range(sizethres[1]):
            for j in range(sizethres[0]):
                tempp=cv.Get2D(im_thres,i,j)
                imagevector[tt]=tempp[0]
                tt=tt+1
        test=self.nnet.predict(imagevector)
    
        test=numpy.array([[1.,1.,1.,1.,0.,1.,1.,1.,1.,0.,0.,1.,1.,1.,1.]])
        test2=nnet.predict(test)
        temp=test2[1]
        temp2=temp.argmax(axis=1)
        return temp2[0]
            
def main():
    fc.init()
    s = rospy.Service('abbgetconnectors', connectors, fc.run)
    rospy.spin() #keps the node running

if __name__ == '__main__':
    rospy.init_node('findconnectors')
    fc=fc()

    main()

