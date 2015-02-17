#!/usr/bin/env python
import roslib; roslib.load_manifest('planner')
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import *
import random
import copy

class State(object):

    def __init__(self, block, fingers, motor):
        self.block_pose = block
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
            (other.block_pose.pose.orientation.w == self.block_pose.pose.orientation.w)

    def __str__(self):
        rep = "block_pose: " + str(self.block_pose) + "\nfinger_angles: " + str(self.finger_angles) + "\nmotor_angle: " + str(self.motor_angle)
        return rep


class Update(object):

    def __init__(self, prob=1, *args, **kwargs):
        self.prob = prob

        self.frame = kwargs.get('frame', None)
        self.x = kwargs.get('x', None)
        self.y = kwargs.get('y', None)
        self.z = kwargs.get('z', None)
        self.q0 = kwargs.get('q0', None)
        self.q1 = kwargs.get('q1', None)
        self.q2 = kwargs.get('q2', None)
        self.q3 = kwargs.get('q3', None)
        self.fingers = kwargs.get('fingers', None)
        self.motor = kwargs.get('motor', None)

        self.block = kwargs.get('block', None)
        self.fingers = kwargs.get('fingers', None)

    def apply(self, s):
        state = copy.deepcopy(s)
        if self.frame != None:
            state.block_pose.header.frame_id = self.frame
        if self.x != None:
            state.block_pose.pose.position.x = self.x
        if self.y != None:
            state.block_pose.pose.position.y = self.y
        if self.z != None:
            state.block_pose.pose.position.z = self.z
        if self.q0 != None:
            state.block_pose.pose.orientation.w = self.q0
        if self.q1 != None:
            state.block_pose.pose.orientation.w = self.q1
        if self.q2 != None:
            state.block_pose.pose.orientation.w = self.q2
        if self.q3 != None:
            state.block_pose.pose.orientation.w = self.q3
        if self.fingers != None:
            state.finger_angles = self.fingers
        if self.motor != None:
            state.motor_angle = self.motor

        if self.block != None:
            state.block_pose = self.block

        return state

    def test(self, state):
        #print "STATE IN TEST: \n" + str(state)
        
        if (self.frame != None) and (state.block_pose.header.frame_id != self.frame):
            return False
        if (self.x != None) and (state.block_pose.pose.position.x != self.x):
            return False
        if (self.y != None) and (state.block_pose.pose.position.y != self.y):
            return False
        if (self.z != None) and (state.block_pose.pose.position.z != self.z):
            return False
        if (self.q0 != None) and (state.block_pose.pose.orientation.q0 != self.q0):
            return False
        if (self.q1 != None) and (state.block_pose.pose.orientation.q1 != self.q1):
            return False
        if (self.q2 != None) and (state.block_pose.pose.orientation.q2 != self.q2):
            return False
        if (self.q3 != None) and (state.block_pose.pose.orientation.q3 != self.q3):
            return False
        if (self.fingers != None) and (state.finger_angles != self.fingers):
            return False
        if (self.motor != None) and (state.motor_angle != self.motor):
            return False
        if (self.block != None): # and (state.block_pose != self.block):
            if ((state.block_pose.header.frame_id != self.block.header.frame_id) or
            (state.block_pose.pose.position.x != self.block.pose.position.x) or
            (state.block_pose.pose.position.y != self.block.pose.position.y) or
            (state.block_pose.pose.position.z != self.block.pose.position.z) or
            (state.block_pose.pose.orientation.x != self.block.pose.orientation.x) or
            (state.block_pose.pose.orientation.y != self.block.pose.orientation.y) or
            (state.block_pose.pose.orientation.z != self.block.pose.orientation.z) or
            (state.block_pose.pose.orientation.w != self.block.pose.orientation.w)):
                return False

        return True
