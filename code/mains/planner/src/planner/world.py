#!/usr/bin/env python
import roslib; roslib.load_manifest('planner')
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import *
import random

from search_new import *
from action_new import *
from state_new import *
from plan_new import *

class World:

    def __init__(self, start_state, actions):
        self.state = start_state
        self.actions = actions
        self.planner = MC(self)

    def execute(self, action):
        self.state = action.execute(self.state)

    def plan(self, goal):
        plan = self.planner.findPlan(self.state, goal)
        print "plan: " + str(plan)

        # for a in plan.actions:
        #     print "step: " + str(a)


        # print "Probability of success of plan: "+str(prob)
        # for a in plan:
        #     self.execute(a)
