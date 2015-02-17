#!/usr/bin/env python
import roslib; roslib.load_manifest('planner')
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import *
import random
import copy

from state_new import *

class Action:

    def __init__(self, name, preconditions, transitions):
        self.name = name
        self.preconditions = preconditions
        self.transitions = transitions

        # check that the transitions probabilities sum to 1
        s = 0
        for t in self.transitions:
            s += t.prob
        if s != 1:
            print "ERROR: TRANSITIONS DO NOT SUM TO 1"

    def __str__(self):
        return self.name

    def getSuccess(self):
        t = max(self.transitions, key=lambda t:t.prob)
        return t

    def getTransition(self):
        running = 0
        p = random.random()
        for t in self.transitions:
            running = running + t.prob
            if p <= running:
                return t     

    def getAllPossibleSuccessors(self, state):
        if not self.preconditions(state):
            return None

        r = []
        for t in self.transitions:
            r.append(start.update(t))

        return r

    def getSuccessSuccessor(self, state):
        return state.update(self.getSuccess())

# move: params=[x,y,z,q0,q1,q2,q3]
def moveSuccess(state, params):
    if state.block_pose.header.frame_id == "\tcp":
        state.block_pose.pose.position.x = params[0]
        state.block_pose.pose.position.y = params[1]
        state.block_pose.pose.position.z = params[2]
        state.block_pose.pose.orientation.w = params[3]
        state.block_pose.pose.orientation.x = params[4]
        state.block_pose.pose.orientation.y = params[5]
        state.block_pose.pose.orientation.z = params[6]
        state.hand_pose.pose.position.x = params[0]
        state.hand_pose.pose.position.y = params[1]
        state.hand_pose.pose.position.z = params[2]
        state.hand_pose.pose.orientation.w = params[3]
        state.hand_pose.pose.orientation.x = params[4]
        state.hand_pose.pose.orientation.y = params[5]
        state.hand_pose.pose.orientation.z = params[6]
    else:
        state.hand_pose.pose.position.x = params[0]
        state.hand_pose.pose.position.y = params[1]
        state.hand_pose.pose.position.z = params[2]
        state.hand_pose.pose.orientation.w = params[3]
        state.hand_pose.pose.orientation.x = params[4]
        state.hand_pose.pose.orientation.y = params[5]
        state.hand_pose.pose.orientation.z = params[6]
movePre = lambda x: True
moveTransitionSuccess = Transition(1, moveSuccess) 
move = Action("move", movePre, [moveTransitionSuccess])

# pick: params=[q0,q1,q2,q3,close_angle,motor_angle]
def pickSuccess(state, params):
    # TEMP: offsets for the position of a block in the hand FINGERTIP
    x_off = 10
    y_off = 10
    z_off = 10
    state.block_pose.header.frame_id = "\tcp"
    state.block_pose.pose.position.x = x_off
    state.block_pose.pose.position.y = y_off
    state.block_pose.pose.position.z = z_off
    state.block_pose.pose.orientation.w = params[0]
    state.block_pose.pose.orientation.x = params[1]
    state.block_pose.pose.orientation.y = params[2]
    state.block_pose.pose.orientation.z = params[3]
    state.finger_angles = (params[4],params[4],params[4])
    state.motor_angle = params[5]
def pickFail(state,params):
    pass

pickPre = lambda x: x.block_pose.header.frame_id == "\world"
pickTransitionSuccess = Transition(0.9, pickSuccess)
pickTransitionFail = Transition(0.1, pickFail)
pick = Action("pick", pickPre, [pickTransitionSuccess, pickTransitionFail])

# place: params=[object_pose(in world frame),finger_angle,motor_angle]
def placeSuccess(state, params):
    state.block_pose.header.frame_id = "\world" 
    state.block_pose.pose.position.x = params[0]
    state.block_pose.pose.position.y = params[1]
    state.block_pose.pose.position.z = params[2]
    state.block_pose.pose.orientation.w = params[3]
    state.block_pose.pose.orientation.x = params[4]
    state.block_pose.pose.orientation.y = params[5]
    state.block_pose.pose.orientation.z = params[6]
    state.finger_angles = (params[7],params[7],params[7])
    state.motor_angle = params[8]    
def placeFail(state, params):
    pass

placePre = lambda x: x.block_pose.header.frame_id == "\tcp"
placeTransitionSuccess = Transition(0.9, placeSuccess)
placeTransitionFail = Transition(0.1, placeFail)
place = Action("place", placePre, [placeTransitionSuccess, placeTransitionFail])

# throw_to_palm: params=[finger_angle,motor_angle]
def throwToPalmSuccess(state, params):
    # TEMP: offsets for the position of a block in the hand ENVELOPING
    x_off = 0
    y_off = 0
    z_off = 0
    state.finger_angles = (params[0],params[0],params[0])
    state.motor_angle = params[1]
def throwToPalmFail(state, params):       
    pass

def throwToPalmPre(state):
    lower_fingertip_angle = 10
    upper_fingertip_angle = 50
    f = sum(state.finger_angles)/len(state.finger_angles)
    if (lower_fingertip_angle > f) or (f > upper_fingertip_angle):
        return False

    # TEMP: offsets for the position of a block in the hand FINGERTIP
    x_off = 10
    y_off = 10
    z_off = 10
    suc = PoseStamped()
    suc.header.frame_id = "\tcp"
    suc.pose.position.x = x_off
    suc.pose.position.y = y_off
    suc.pose.position.z = z_off
    suc.pose.orientation.w = 0.707
    suc.pose.orientation.w = 0.707
    suc.pose.orientation.w = 0.0
    suc.pose.orientation.w = 0.0
    if not sameCluster(state.block_pose, suc):
        return False


throwToPalmTransitionSuccess = Transition(0.9, throwToPalmSuccess)
throwToPalmTransitionFail = Transition(0.1, throwToPalmFail)
throwToPalm = Action("throwToPalm", throwToPalmPre, 
                     [throwToPalmTransitionSuccess, throwToPalmTransitionFail])


def sameCluster(pose1, pose2):
    t = 0.2
    if (pose1.header.frame_id == pose2.header.frame_id) and \
            (pose1.pose.orientation.w == pose2.pose.orientation.w) and \
            (pose1.pose.orientation.x == pose2.pose.orientation.x) and \
            (pose1.pose.orientation.y == pose2.pose.orientation.y) and \
            (pose1.pose.orientation.z == pose2.pose.orientation.z) and \
            ((pose1.pose.position.x - pose2.position.x) <= t) and \
            ((pose1.pose.position.y - pose2.position.y) <= t) and \
            ((pose1.pose.position.z - pose2.position.z) <= t):
        return True
    return False
