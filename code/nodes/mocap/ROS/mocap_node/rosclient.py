#!/usr/bin/env python
import roslib
roslib.load_manifest('mocap_node')
import rospy

from std_msgs.msg import String

pos = "hi" 

def callback(data):
    f = open('data.txt', 'w')
    f.write(str(data.data))
    f.close()
    
    print data.data
def listen():
    
    rospy.init_node('listen', anonymous=True)
    rospy.Subscriber("mocap_marker_data", String, callback)
    rospy.spin()


listen() 
