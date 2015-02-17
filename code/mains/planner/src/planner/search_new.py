#!/usr/bin/env python
import roslib; roslib.load_manifest('planner')
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import *
import random
import operator

from world import *
from action_new import *
from plan_new import *
from state_new import *
from tools import *

class MC:

    def __init__(self, world):
        self.world = world

    def findPlan(self, start, goal):
        plans = []

        for i in range(0,50):
            plan = self.rollout(start, goal)

            if plan not in plans:
                plans.append(plan)
            for existing in plans:
                if existing == plan:
                    # if plan.getReward() != 0:
                    #     print "plan: "+str(plan)+" r: "+str(plan.getReward())
                    avg = (plan.getReward() + existing.getReward()) / 2
                    plan.setReward(avg)

        for plan in plans:
            print "possible plan: "+str(plan)+" r: "+str(plan.getReward())
    
        return max(plans, key=lambda p:p.getReward())

    def rollout(self, state, goal):
        plan = Plan()
        for i in range(0, 5):
            if state == goal:
                plan.setReward(plan.planProb())
                return plan

            action = self.step(state)
            params = []
            t = action.getTransition()
            print "STATE: "+str(state)
            state.test(1)
            state.update(t.fn, params) #action.execute(state)
            # if not action.succeeded(state, tstate):
            #     print "action "+str(action)+" failed"
            plan.add(action)

        if state == goal:
            plan.setReward(plan.planProb())
            return plan
        
        return plan

    def getPossibleActions(self, state):
        result = []
        for a in self.world.actions:
            if a.preconditions(state):
                result.append(a)
        return result

    def step(self, state): # choose an action based on prob of success
        actions = self.getPossibleActions(state)

        p = random.random()
        pr = 0
        for a in actions:
            pr += a.getSuccess().prob

        running = 0
        for a in actions:
            running += a.getSuccess().prob/pr
            if p <= running:
                return a
