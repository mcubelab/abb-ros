#!/usr/bin/env python
import roslib; roslib.load_manifest('planner')
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import *
import random

# from world import *
# from action import *
# from state import *
# from search import *
# from tools import *

from world import *
from action_new import *
from state_new import *
from search_new import *
from plan_new import *
from tools import *

if __name__ == '__main__':

    # # define some actions
    # # TRIANGLE
    # move = Action("move", move_pre, [move_success])
    # pick = Action("pick", pick_pre, [pick_success, pick_fail])
    # place = Action("place", place_pre, [place_success, place_fail])
    # roll_to_ground = Action("roll_to_ground", roll_to_ground_pre, 
    #                         [roll_to_ground_success, roll_to_ground_fail])
    # roll_to_palm = Action("roll_to_palm", roll_to_palm_pre, 
    #                       [roll_to_palm_success, roll_to_palm_fail])
    # throw_to_palm = Action("throw_to_palm", throw_to_palm_pre, \
    #                            [throw_to_palm_success, throw_to_palm_fail])
    # throw_to_fingertip = Action("throw_to_fingertip", throw_to_fingertip_pre, 
    #                             [throw_to_fingertip_success, throw_to_fingertip_fail])
    # throw_and_flip = Action("throw_and_flip", throw_and_flip_pre, 
    #                         [throw_and_flip_success, throw_and_flip_fail])

    # # CYLINDER
    # droop = Action("droop_in_fingers", droop_pre, 
    #                [droop_success, droop_fail_tight, droop_fail_loose])
    # roll_to_fingertip = Action("roll_to_fingertip", roll_to_fingertip_pre,
    #                            [roll_to_fingertip_success, \
    #                                 roll_to_fingertip_fail_short,
    #                             roll_to_fingertip_fail_long])
    # push_in_fingers = Action("push_in_fingers", push_in_fingers_pre, 
    #                          [push_in_fingers_success, push_in_fingers_fail])
    # push_in_enveloping = Action("push_in_enveloping", push_in_enveloping_pre, 
    #                             [push_in_enveloping_success, push_in_enveloping_fail])
    # roll_on_ground = Action("roll_on_ground", roll_on_ground_pre, 
    #                         [roll_on_ground_success, roll_on_ground_fail])

    # triangle_actions = [pick, place, throw_to_palm, throw_to_fingertip, 
    #                     throw_and_flip, roll_to_palm, roll_to_ground]

    # cylinder_actions = [pick, place, roll_to_fingertip, push_in_fingers, 
    #                     push_in_enveloping, roll_on_ground]

    # # CREATE ENVIRONMENT FOR THE TRIANGLE
    
    # start = State(ground_center_flip_x, open_hand, motor_open)
    # # pick, throw_to_palm
    # goal1 = State(hand_center_flip_x, closed_enveloping_hand, motor_hard)
    # # pick, roll_to_palm
    # goal2 = State(hand_center, closed_enveloping_hand, motor_hard)
    # # pick, roll_to_palm, place, pick
    # goal3 = State(hand_center, closed_fingertip_hand, motor_hard)

    start = State(ground_center_flip_x, ground_center_flip_x, open_hand, motor_open)
    goal = State(hand_center_flip_x, hand_center_flip_x, closed_fingertip_hand, motor_hard)
    triangle_actions = [move, pick, place, throwToPalm]

    env = World(start, triangle_actions)
    env.plan(goal)

    # print "TESTS"
    # pickadj = pick.adjacent(triangle_actions, start)
    # placeadj = place.adjacent(triangle_actions, goal3)
    # for t in pickadj:
    #     print "pick: "+str(t[0])
    # for t in placeadj:
    #     print "place: "+str(t[0])
    
