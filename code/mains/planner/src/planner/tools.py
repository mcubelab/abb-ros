#!/usr/bin/env python
import roslib; roslib.load_manifest('planner')
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import *

from state_new import *

# SAMPLE POSES
# ground/hand, center/pos/neg, --/flip_x/flip_y

poses = []

# vary the position
stepsize = 10
inc = [i/float(stepsize) for i in range(0,stepsize)]

# vary the orientation (90 deg on each axis)
q0 = Quaternion()
q0.w = 1.0
q0.x = 0.0
q0.y = 0.0
q0.z = 0.0
q1 = Quaternion()
q1.w = 0.707
q1.x = 0.707
q1.y = 0.0
q1.z = 0.0
q2 = Quaternion()
q2.w = 0.707
q2.x = 0.0
q2.y = 0.707
q2.z = 0.0
q3 = Quaternion()
q3.w = 0.707
q3.x = 0.0
q3.y = 0.0
q3.z = 0.707
qs = [q0,q1,q2,q3]

for x in inc:
    for y in inc:
        for z in inc:
            for q in qs:
                pose = PoseStamped()
                pose.header.frame_id = "/world"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = z
                pose.pose.orientation = q
    
                poses.append(pose)

# POSES FOR TESTING
ground_center = PoseStamped()
ground_center.header.frame_id = "/world"
ground_center.pose.position.x = 0
ground_center.pose.position.y = 0
ground_center.pose.position.z = 0
ground_center.pose.orientation.w = 1
ground_center.pose.orientation.x = 0
ground_center.pose.orientation.y = 0
ground_center.pose.orientation.z = 0

ground_pos = PoseStamped()
ground_pos.header.frame_id = "/world"
ground_pos.pose.position.x = 5
ground_pos.pose.position.y = 5
ground_pos.pose.position.z = 5
ground_pos.pose.orientation.w = 1
ground_pos.pose.orientation.x = 0
ground_pos.pose.orientation.y = 0
ground_pos.pose.orientation.z = 0

ground_neg = PoseStamped()
ground_neg.header.frame_id = "/world"
ground_neg.pose.position.x = -5
ground_neg.pose.position.y = -5
ground_neg.pose.position.z = -5
ground_neg.pose.orientation.w = 1
ground_neg.pose.orientation.x = 0
ground_neg.pose.orientation.y = 0
ground_neg.pose.orientation.z = 0

ground_center_flip_x = PoseStamped()
ground_center_flip_x.header.frame_id = "/world"
ground_center_flip_x.pose.position.x = 0
ground_center_flip_x.pose.position.y = 0
ground_center_flip_x.pose.position.z = 0
ground_center_flip_x.pose.orientation.w = 0.707
ground_center_flip_x.pose.orientation.x = 0.707
ground_center_flip_x.pose.orientation.y = 0
ground_center_flip_x.pose.orientation.z = 0

ground_pos_flip_x = PoseStamped()
ground_pos_flip_x.header.frame_id = "/world"
ground_pos_flip_x.pose.position.x = 5
ground_pos_flip_x.pose.position.y = 5
ground_pos_flip_x.pose.position.z = 5
ground_pos_flip_x.pose.orientation.w = 0.707
ground_pos_flip_x.pose.orientation.x = 0.707
ground_pos_flip_x.pose.orientation.y = 0
ground_pos_flip_x.pose.orientation.z = 0

ground_neg_flip_x = PoseStamped()
ground_neg_flip_x.header.frame_id = "/world"
ground_neg_flip_x.pose.position.x = -5
ground_neg_flip_x.pose.position.y = -5
ground_neg_flip_x.pose.position.z = -5
ground_neg_flip_x.pose.orientation.w = 0.707
ground_neg_flip_x.pose.orientation.x = 0.707
ground_neg_flip_x.pose.orientation.y = 0
ground_neg_flip_x.pose.orientation.z = 0

ground_center_flip_y = PoseStamped()
ground_center_flip_y.header.frame_id = "/world"
ground_center_flip_y.pose.position.x = 0
ground_center_flip_y.pose.position.y = 0
ground_center_flip_y.pose.position.z = 0
ground_center_flip_y.pose.orientation.w = 0.707
ground_center_flip_y.pose.orientation.x = 0
ground_center_flip_y.pose.orientation.y = 0.707
ground_center_flip_y.pose.orientation.z = 0

ground_pos_flip_y = PoseStamped()
ground_pos_flip_y.header.frame_id = "/world"
ground_pos_flip_y.pose.position.x = 5
ground_pos_flip_y.pose.position.y = 5
ground_pos_flip_y.pose.position.z = 5
ground_pos_flip_y.pose.orientation.w = 0.707
ground_pos_flip_y.pose.orientation.x = 0
ground_pos_flip_y.pose.orientation.y = 0.707
ground_pos_flip_y.pose.orientation.z = 0

ground_neg_flip_y = PoseStamped()
ground_neg_flip_y.header.frame_id = "/world"
ground_neg_flip_y.pose.position.x = -5
ground_neg_flip_y.pose.position.y = -5
ground_neg_flip_y.pose.position.z = -5
ground_neg_flip_y.pose.orientation.w = 0.707
ground_neg_flip_y.pose.orientation.x = 0
ground_neg_flip_y.pose.orientation.y = 0.707
ground_neg_flip_y.pose.orientation.z = 0

hand_center = PoseStamped()
hand_center.header.frame_id = "/tcp"
hand_center.pose.position.x = 0
hand_center.pose.position.y = 0
hand_center.pose.position.z = 0
hand_center.pose.orientation.w = 1
hand_center.pose.orientation.x = 0
hand_center.pose.orientation.y = 0
hand_center.pose.orientation.z = 0

hand_pos = PoseStamped()
hand_pos.header.frame_id = "/tcp"
hand_pos.pose.position.x = 5
hand_pos.pose.position.y = 5
hand_pos.pose.position.z = 5
hand_pos.pose.orientation.w = 1
hand_pos.pose.orientation.x = 0
hand_pos.pose.orientation.y = 0
hand_pos.pose.orientation.z = 0

hand_neg = PoseStamped()
hand_neg.header.frame_id = "/tcp"
hand_neg.pose.position.x = -5
hand_neg.pose.position.y = -5
hand_neg.pose.position.z = -5
hand_neg.pose.orientation.w = 1
hand_neg.pose.orientation.x = 0
hand_neg.pose.orientation.y = 0
hand_neg.pose.orientation.z = 0

hand_center_flip_x = PoseStamped()
hand_center_flip_x.header.frame_id = "/tcp"
hand_center_flip_x.pose.position.x = 0
hand_center_flip_x.pose.position.y = 0
hand_center_flip_x.pose.position.z = 0
hand_center_flip_x.pose.orientation.w = 0.707
hand_center_flip_x.pose.orientation.x = 0.707
hand_center_flip_x.pose.orientation.y = 0
hand_center_flip_x.pose.orientation.z = 0

hand_pos_flip_x = PoseStamped()
hand_pos_flip_x.header.frame_id = "/tcp"
hand_pos_flip_x.pose.position.x = 5
hand_pos_flip_x.pose.position.y = 5
hand_pos_flip_x.pose.position.z = 5
hand_pos_flip_x.pose.orientation.w = 0.707
hand_pos_flip_x.pose.orientation.x = 0.707
hand_pos_flip_x.pose.orientation.y = 0
hand_pos_flip_x.pose.orientation.z = 0

hand_neg_flip_x = PoseStamped()
hand_neg_flip_x.header.frame_id = "/tcp"
hand_neg_flip_x.pose.position.x = -5
hand_neg_flip_x.pose.position.y = -5
hand_neg_flip_x.pose.position.z = -5
hand_neg_flip_x.pose.orientation.w = 0.707
hand_neg_flip_x.pose.orientation.x = 0.707
hand_neg_flip_x.pose.orientation.y = 0
hand_neg_flip_x.pose.orientation.z = 0

hand_center_flip_y = PoseStamped()
hand_center_flip_y.header.frame_id = "/tcp"
hand_center_flip_y.pose.position.x = 0
hand_center_flip_y.pose.position.y = 0
hand_center_flip_y.pose.position.z = 0
hand_center_flip_y.pose.orientation.w = 0.707
hand_center_flip_y.pose.orientation.x = 0
hand_center_flip_y.pose.orientation.y = 0.707
hand_center_flip_y.pose.orientation.z = 0

hand_pos_flip_y = PoseStamped()
hand_pos_flip_y.header.frame_id = "/tcp"
hand_pos_flip_y.pose.position.x = 5
hand_pos_flip_y.pose.position.y = 5
hand_pos_flip_y.pose.position.z = 5
hand_pos_flip_y.pose.orientation.w = 0.707
hand_pos_flip_y.pose.orientation.x = 0
hand_pos_flip_y.pose.orientation.y = 0.707
hand_pos_flip_y.pose.orientation.z = 0

hand_neg_flip_y = PoseStamped()
hand_neg_flip_y.header.frame_id = "/tcp"
hand_neg_flip_y.pose.position.x = -5
hand_neg_flip_y.pose.position.y = -5
hand_neg_flip_y.pose.position.z = -5
hand_neg_flip_y.pose.orientation.w = 0.707
hand_neg_flip_y.pose.orientation.x = 0
hand_neg_flip_y.pose.orientation.y = 0.707
hand_neg_flip_y.pose.orientation.z = 0

# SAMPLE FINGER_ANGLES
open_hand = (10,10,10)
closed_enveloping_hand = (60,60,60)
closed_fingertip_hand = (30,30,30)

# SAMPLE MOTOR_ANGLES
motor_open = 90
motor_light = 50
motor_hard = 10

# ACTION PRECONDITIONS
# move_pre = Update(frame="/tcp")

# pick_pre = Update(frame="/world")

# place_pre = Update(frame="/tcp")

# droop_pre = Update(frame="/world")

# throw_to_palm_pre = Update(block=hand_center_flip_x, 
#                            fingers=closed_fingertip_hand, motor=motor_hard)

# throw_to_fingertip_pre = Update(block=hand_center_flip_x,
#                                 fingers=closed_enveloping_hand,
#                                 motor=motor_hard)

# throw_and_flip_pre = Update(block=hand_center_flip_x, fingers=closed_enveloping_hand,
#                             motor=motor_hard)

# roll_to_palm_pre = Update(block=hand_center_flip_x, 
#                           fingers=closed_fingertip_hand, motor=motor_hard)

# roll_to_fingertip_pre = Update(block=hand_center, 
#                                fingers=closed_enveloping_hand)

# roll_to_ground_pre = Update(block=hand_center_flip_x, 
#                             fingers=closed_fingertip_hand)

# push_in_fingers_pre = Update(block=hand_center, fingers=closed_fingertip_hand)

# push_in_enveloping_pre = Update(frame="/tcp", fingers=closed_enveloping_hand)

# roll_on_ground_pre = Update(frame="/tcp", fingers=closed_fingertip_hand)

# # ACTION UPDATES
# move_success = Update(1.0, )

# pick_success = Update(0.99, frame="/tcp", fingers=closed_fingertip_hand,
#                       motor=motor_hard)
# pick_fail = Update(0.01)

# place_success = Update(0.99, frame="/world", fingers=open_hand, 
#                        motor=motor_open)
# place_fail = Update(0.01)

# droop_success = Update(0.7, block=hand_center_flip_x, 
#                        fingers=closed_fingertip_hand, motor=motor_light)
# droop_fail_tight = Update(0.15, fingers=closed_fingertip_hand, motor=motor_light)
# droop_fail_loose = Update(0.15)

# throw_to_palm_success = Update(0.8, fingers=closed_enveloping_hand)
# throw_to_palm_fail = Update(0.2)

# throw_to_fingertip_success = Update(0.7, fingers=closed_fingertip_hand)
# throw_to_fingertip_fail = Update(0.3)

# throw_and_flip_success = Update(0.7, block=hand_center)
# throw_and_flip_fail = Update(0.3)

# roll_to_palm_success = Update(0.95, block=hand_center, 
#                               fingers=closed_enveloping_hand)
# roll_to_palm_fail = Update(0.05, fingers=closed_enveloping_hand)

# roll_to_fingertip_success = Update(0.9, fingers=closed_fingertip_hand)
# roll_to_fingertip_fail_short = Update(0.05)
# roll_to_fingertip_fail_long = Update(0.05, fingers=open_hand, motor=motor_open)

# roll_to_ground_success = Update(0.9, block=ground_center, 
#                                 fingers=open_hand, motor=motor_open)
# roll_to_ground_fail = Update(0.1, fingers=open_hand, motor=motor_open)

# push_in_fingers_success = Update(0.95, block=hand_center_flip_x)
# push_in_fingers_fail = Update(0.05)

# push_in_enveloping_success = Update(0.99, block=hand_center)
# push_in_enveloping_fail = Update(0.01)

# roll_on_ground_success = Update(0.99, q0=0.0707,q1=0.707)
# roll_on_ground_fail = Update(0.01)


