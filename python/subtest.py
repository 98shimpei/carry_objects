#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers
rospy.init_node('test_sub')
def cb(msg):
    print("sub")

rospy.Subscriber("test", AlvarMarkers, cb)
rospy.spin()
