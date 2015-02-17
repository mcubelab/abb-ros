#!/usr/bin/env python
import roslib; roslib.load_manifest('planner')
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import *
import random
import copy

class State(object):

    def __init__(self, block, hand, fingers, motor):
        # the block pose is in the frame of the world when its not held and in the 
        # tcp frame when it is in the hand
        self.block_pose = block
        self.hand_pose = hand
        self.finger_angles = fingers
        self.motor_angle = motor

    def __eq__(self, other):
        return isinstance(other, self.__class__)  and \
            (self.motor_angle == other.motor_angle) and \
            (self.finger_angles == other.finger_angles) and \
            (other.block_pose.header.frame_id == self.block_pose.header.frame_id) and \
            (other.block_pose.pose.position.x == self.block_pose.pose.position.x) and \
            (other.block_pose.pose.position.y == self.block_pose.pose.position.y) and \
            (other.block_pose.pose.position.z == self.block_pose.pose.position.z) and \
            (other.block_pose.pose.orientation.x == self.block_pose.pose.orientation.x) and \
            (other.block_pose.pose.orientation.y == self.block_pose.pose.orientation.y) and \
            (other.block_pose.pose.orientation.z == self.block_pose.pose.orientation.z) and \
            (other.block_pose.pose.orientation.w == self.block_pose.pose.orientation.w) and \
            (other.block_pose.header.frame_id == self.block_pose.header.frame_id) and \
            (other.block_pose.pose.position.x == self.block_pose.pose.position.x) and \
            (other.block_pose.pose.position.y == self.block_pose.pose.position.y) and \
            (other.block_pose.pose.position.z == self.block_pose.pose.position.z) and \
            (other.block_pose.pose.orientation.x == self.block_pose.pose.orientation.x) and \
            (other.block_pose.pose.orientation.y == self.block_pose.pose.orientation.y) and \
            (other.block_pose.pose.orientation.z == self.block_pose.pose.orientation.z) and \
            (other.block_pose.pose.orientation.w == self.block_pose.pose.orientation.w)

    def __str__(self):
        return "block_pose: " + str(self.block_pose) + \
            "\nhand_pose: "+ str(self.hand_pose) + \
            "\nfinger_angles: " + str(self.finger_angles) + \
            "\nmotor_angle: " + str(self.motor_angle)

    def update(self, fn, params):
        fn(self, params)

    def test(self, x):
        print "x: "+str(x)

class Transition:
    
    def __init__(self, prob, fn):
        self.prob = prob
        self.fn = fn

