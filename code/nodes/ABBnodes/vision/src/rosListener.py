#!/usr/bin/env python
import roslib; roslib.load_manifest('vision')
import rospy
from vision.msg import pos
def callback(data):
    #rospy.loginfo(rospy.get_name()+"I heard %s",data.data)
    print data.pos[0]
    print data.pos[1]
def listener():
    rospy.init_node('rosListener', anonymous=True)
    sub = rospy.Subscriber("Battery_pos", pos, callback)
    print pos
    rospy.spin()

if __name__ == '__main__':
    listener()
