#!/usr/bin/env python
import roslib; roslib.load_manifest('planner')
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import *
import random
import copy

from state_new import *
from action_new import *

class Plan:

    def __init__(self):
        self.actions = []
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

    def add(self, a):
        self.actions.append(a)

    def insert(self, i, a):
        self.actions.insert(i, a)

    def execute(self, start):
        state = copy.deepcopy(start)

        for a in self.actions:
            state.update(a.getTransition())

        return state

    def success(self, start):
        state = copy.deepcopy(start)
        
        for a in self.actions:
            state.update(a.getSuccess)

        return state

    def planProb(self):
        prob = 1
        for a in self.actions:
            prob = prob * a.getSuccess().prob

        return prob
