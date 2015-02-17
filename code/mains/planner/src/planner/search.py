#!/usr/bin/env python
import roslib; roslib.load_manifest('planner')
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import *
import random
import operator

from world import *
from action import *
from state import *
from tools import *

class Search:

    def __init__(self, world):
        self.world = world

    # state is a set of conditions imposed by the action

    def getPossibleActions(self, state):
        result = []
        for a in self.world.actions:
            #if a.name == "throw_to_palm":
                #print "test: "+ str(a.preconditions.test(state))
                #print "STATE: "+str(state)
                #print "PRE: "+str(hand_center_flip_x)

            if a.preconditions.test(state):
                result.append(a)
        return result

    def step(self, state): # choose an action based on prob of success
        actions = self.getPossibleActions(state)

        p = random.random()
        pr = 0
        for a in actions:
            pr += a.getSuccessProb()

        running = 0
        for a in actions:
            running += a.getSuccessProb()/pr
            if p <= running:
                return a

    def rollout(self, state, goal):
        plan = Plan()
        for i in range(0, 5):
            if state == goal:
                plan.setReward(plan.planProb())
                return plan

            action = self.step(state)
            (tstate, p) = action.execute(state)
            if not action.succeeded(state, tstate):
                print "action "+str(action)+" failed"
            state = tstate
            plan.add(action, p)

        if state == goal:
            plan.setReward(plan.planProb())
            return plan
        
        return plan
    
    def plan(self, start, goal):
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



    # def step(self, plans, state):
    #     for plan in plans:
    #         last_action = plan.actions[-1]
    #         adj = last_action.adjacent(self.world.actions, state)
    #         best = max(adj, key=lambda x:x[1])
    #         plan.add(best[0], best[1])
    #         print "plan: "+str(plan)

    # def testPlans(self, plans, start, goal):
    #     for plan in plans:
    #         if plan.doPlan(start) == goal:
    #             return plan
    #     return None

    # def plan(self, start, goal):
    #     actions = self.getPossibleActions(start)
    #     state = copy.deepcopy(start)
        
    #     plans = []
    #     for a in actions:
    #         plan = Plan(a, 1)
    #         plans.append(plan)
    #         print "plans: "+str(plan)

    #     test = self.testPlans(plans, start, goal)
    #     while test == None:
    #         self.step(plans, state)
    #         test = self.testPlans(plans, start, goal)
    #     return test



        # for a in actions:
        #     print "first a: " + str(a)

        #     plan = Plan()
        #     plan.add(a, 1)
        #     plans.append(plan)

        #     while not plan.metGoal(start, goal):
        #     #for i in range(0,1):
        #         last_action = plan.actions[-1]
        #         adj = last_action.adjacent(self.world.actions, state)
        #         best = max(adj, key=lambda x:x[1])
        #         print "next best: " + str(best[0])
        #         plan.add(best[0], best[1])
        #         state = best[0].child(state)
        #         print "state: "+str(state)

        #     print "done: "+str(plan.metGoal(state,goal))
        #     return plan


#self.flow(start, plan)
        

    # def flow(self, state, plan):
    #     last_action = plan.actions[-1] 
    #     adj = last.adjacent(self.world.actions, state)
    #     best = max(adj, key=lambda a:a[1])[0]
        


    # def plan_bfs(self, start, goal):
    #     prob = 1

    #     queue = []
    #     queue.append([start])

    #     while queue: 
    #         node = queue.pop(0)

    #         if node == goal:
    #             return node

    #         for adjacent in node.actions
    #             new_plan = list(plan)
    #             new_plan.append(adjacent)
    #             queue.append(new_plan)





    #     if start == goal:
    #         return 
    #     while s != goal:
    #         # find possible actions
    #         possible = []
    #         for a in self.world.actions:
    #             for t in a.transitions:
    #                 if t.results(s)

    #                 if a.transitions[t][1] == goal:
    #                     possible.append((a, a.transitions[t][0], a.transitions[t][1]))
    #         if len(possible) == 0:
    #             no_plan = True
    #             return (plan, 0)

    #         # greedily choose safest action
    #         action = max(possible, key=lambda item:item[1])[0]
    #         plan.append(action)
    #         prob = prob * max(possible, key=lambda item:item[1])[1]
    #         print "action: "+str(action.name)+" prob: "+str(prob)
    #         s = action.start_state

    #     # reverse the plan
    #     plan = plan[::-1]
    #     return (plan, prob)
