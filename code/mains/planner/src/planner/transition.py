#!/usr/bin/env python
import roslib; roslib.load_manifest('planner')
import rospy
from std_msgs.msg import String
from geometry_msgs import *
import random

def getTransition(transitions):
    running = 0
    p = random.random()
    for prob in transitions.keys():
        running = running + prob
        if p <= running:
            return transitions[prob]

class HandState:

    block_pose # relative to TCP
    finger_angles
    motor_angle

    def __init__(self, block, fingers, motor):
        block_pose = block
        finger_angles = fingers
        motor_angle = motor        

    def update(self, block, fingers, motor):
        block_pose = block
        finger_angles = fingers
        motor_angle = motor

    def __eq__(self, other):
        return isinstance(other, self.__class__) and self.__dict__ == other.__dict__)

class WorldState:

    block_pose # relative to world frame

    def __init__(self, block):
        block_pose = block

    def update(self, block):
        block_pose = block

    def __eq__(self, other):
        return isinstance(other, self.__class__) and self.__dict__ == other.__dict__)



class Action(object):

    def __init__(self, transitions):
        self.name = name
        self.transitions = transition # dictionary for now

    def getName(self):
        return self.name

    def execute(self, state):
        return getTransition(self.transitions)


class HandToHand(Action):
    pass

class HandToWorld(Action):
    pass

class WorldToHand(Action):
    pass

class WorldToWorld(Action):
    pass

if __name__ == '__main__':

    p0 = Pose()
    p0.position.x = 0
    p0.position.y = 0
    p0.position.z = 0
    p0.orientation.w = 1
    p0.orientation.x = 0
    p0.orientation.y = 0
    p0.orientation.z = 0

    p1 = Pose()
    p1.position.x = 0
    p1.position.y = 0
    p1.position.z = 0
    p1.orientation.w = 0.707
    p1.orientation.x = 0.707
    p1.orientation.y = 0
    p1.orientation.z = 0

    state0 = WorldState(p0)


    droop = Action({0.7:})
