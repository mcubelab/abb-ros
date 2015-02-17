#!/usr/bin/env python
import roslib; roslib.load_manifest('planner')
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import *
import random
import copy

from state import *

class Plan:

    def __init__(self):
        self.actions = []
        self.probs = []
        self.reward = 0

    def __str__(self):
        p = ""
        for a in self.actions:
            p += str(a) + " "
        return p

    def __eq__(self, other):
        return (self.actions == other.actions) 

    def __len__(self):
        return len(self.actions)

    def setReward(self, r):
        self.reward = r

    def getReward(self):
        return self.reward

    def add(self, a, p):
        self.actions.append(a)
        self.probs.append(p)

    def insert(self, i, a, p):
        self.actions.insert(i, a)
        self.probs.insert(i, p) 

    def planProb(self):
        prob = 1
        for p in self.probs:
            prob *= p
        return prob

    def doPlan(self, start):
        state = copy.deepcopy(start)

        for a in self.actions:
            (state, p) = a.child(state) #a.execute(state)
            
        return state

    def metGoal(self, start, goal):
        resulting = self.doPlan(start)
        return resulting == goal

class Action:

    def __init__(self, name, preconditions, transitions):
        self.name = name
        self.preconditions = preconditions
        self.transitions = transitions

        s = 0
        for t in self.transitions:
            s += t.prob
        if s != 1:
            print "ERROR: TRANSITIONS DO NOT SUM TO 1"

    def __str__(self):
        return self.name

    def getSuccessProb(self):
        t = max(self.transitions, key=lambda t:t.prob)
        return t.prob

    def succeeded(self, start, end):
        if self.child(start)[0] == end:
            return True
        else:
            return False
    
    def getTransition(self):
        running = 0
        p = random.random()
        for t in self.transitions:
            running = running + t.prob
            if p <= running:
                return t

    def adjacent(self, actions, state):
        result = []
        # if you execute the current action 
        (child, p) = self.child(state)
        for a in actions:
            if a.preconditions.test(child):
                result.append((a, p))

        return result

    def children(self, start):
        if not self.preconditions.test(start):
            return None

        r = []
        for t in self.transitions:
            r.append((t.apply(start), t.prob))

        return r

    def child(self, start):
        if not self.preconditions.test(start):
            return None

        success = max(self.transitions, key=lambda t:t.prob)
        return (success.apply(start), success.prob)

    def execute(self, start):
        if self.preconditions.test(start):
            t = self.getTransition()
            return (t.apply(start), t.prob) #start.apply(t)
        else:
            return (start, 1)

    def update(self, changes):
        state = copy.deepcopy(self.start_state)
        #print "changes:"+str(changes)
        for k in changes.keys():
            if k == "frame":
                state.block_pose.header.frame_id = changes[k]
            elif k == "x":
                state.block_pose.pose.position.x = changes[k]
            elif k == "y":
                state.block_pose.pose.position.y = changes[k]
            elif k == "z":
                state.block_pose.pose.position.z = changes[k]
            elif k == "q0":
                state.block_pose.pose.orientation.w = changes[k]
            elif k == "q1":
                state.block_pose.pose.orientation.x = changes[k]
            elif k == "q2":
                state.block_pose.pose.orientation.y = changes[k]
            elif k == "q3":
                state.block_pose.pose.orientation.z = changes[k]
            elif k == "finger0":
                state.finger_angles[0] = changes[k]
            elif k == "finger1":
                state.finger_angles[1] = changes[k]
            elif k == "finger2":
                state.finger_angles[2] = changes[k]
            elif k == "motor":
                state.motor_angle = changes[k]
            elif k == "attach":
                state.finger_angles = (60,60,60)
                state.motor_angle = 10 # guess
                state.block_pose.header.frame_id = "/tcp"
                state.block_pose.pose.position.x = changes[k][0]
                state.block_pose.pose.position.y = changes[k][1]
                state.block_pose.pose.position.z = changes[k][2]
                state.block_pose.pose.orientation.w = changes[k][3]
                state.block_pose.pose.orientation.x = changes[k][4]
                state.block_pose.pose.orientation.y = changes[k][5]
                state.block_pose.pose.orientation.z = changes[k][6]
            elif k == "dettach":
                state.finger_angles = (10,10,10)
                state.motor_angle = 90 # guess
                state.block_pose.header.frame_id = "/world"
                state.block_pose.pose.position.x = changes[k][0]
                state.block_pose.pose.position.y = changes[k][1]
                state.block_pose.pose.position.z = changes[k][2]
                state.block_pose.pose.orientation.w = changes[k][3]
                state.block_pose.pose.orientation.x = changes[k][4]
                state.block_pose.pose.orientation.y = changes[k][5]
                state.block_pose.pose.orientation.z = changes[k][6]

        return state


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
